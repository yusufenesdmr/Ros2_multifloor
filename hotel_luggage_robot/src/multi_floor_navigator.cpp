/**
 * multi_floor_navigator.cpp
 *
 * ROS2 port of the Multi-Map-Navigation system (originally ROS1/Catkin).
 * Implements a lifecycle-aware action server that:
 *   1. Accepts LuggageDelivery action goals (pickup + drop-off across floors)
 *   2. Queries SQLite wormhole DB for elevator waypoints between floors
 *   3. Navigates to each waypoint via Nav2 NavigateToPose action
 *   4. Calls the elevator controller service to request floor changes
 *   5. Switches map contexts using Nav2 LifecycleManager / LoadMap service
 *   6. Monitors nearby persons detected by person_detector node and
 *      pauses/slows when humans are within the safety radius
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hotel_luggage_robot/action/luggage_delivery.hpp"
#include "hotel_luggage_robot/msg/detected_person_array.hpp"
#include "hotel_luggage_robot/srv/call_elevator.hpp"

#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>

using namespace std::chrono_literals;
using LuggageDelivery  = hotel_luggage_robot::action::LuggageDelivery;
using GoalHandleLD     = rclcpp_action::ServerGoalHandle<LuggageDelivery>;
using DetectedPersonArray = hotel_luggage_robot::msg::DetectedPersonArray;
using CallElevator     = hotel_luggage_robot::srv::CallElevator;
using NavigateToPose   = nav2_msgs::action::NavigateToPose;
using LoadMap          = nav2_msgs::srv::LoadMap;

// ─── Wormhole (elevator) entry point ──────────────────────────────────────────
struct Wormhole {
    std::string from_floor;
    std::string to_floor;
    double approach_x;          // robot navigates here before calling elevator
    double approach_y;
    double approach_yaw;
    std::string elevator_id;
    std::string map_yaml_path;  // YAML for the target floor's map
};

// ═══════════════════════════════════════════════════════════════════════════════
class MultiFloorNavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
    MultiFloorNavigator()
    : rclcpp_lifecycle::LifecycleNode("multi_floor_navigator")
    {
        declare_parameter("wormhole_db_path", "hotel_wormholes.db");
        declare_parameter("maps_base_path",   "maps/");
        declare_parameter("initial_floor",    "floor_1");
        declare_parameter("person_safety_radius_m",  1.5);
        declare_parameter("person_pause_timeout_sec", 10.0);
        declare_parameter("nav2_navigate_action",
            "/navigate_to_pose");
        declare_parameter("elevator_service",
            "/elevator_controller/call_elevator");
        declare_parameter("amcl_initial_pose_topic",
            "/initialpose");
        declare_parameter("map_load_service",
            "/map_server/load_map");
        declare_parameter("lifecycle_manager_service",
            "/lifecycle_manager_navigation/manage_nodes");
    }

    // ─── Lifecycle callbacks ─────────────────────────────────────────────────
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Configuring MultiFloorNavigator ...");

        current_floor_ = get_parameter("initial_floor").as_string();
        db_path_       = get_parameter("wormhole_db_path").as_string();

        if (!open_database()) {
            RCLCPP_ERROR(get_logger(), "Cannot open wormhole database: %s", db_path_.c_str());
            return CallbackReturn::FAILURE;
        }

        // Subscribers
        persons_sub_ = create_subscription<DetectedPersonArray>(
            "/detected_persons", 10,
            [this](const DetectedPersonArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(persons_mutex_);
                latest_persons_ = *msg;
            });

        // Publisher: broadcast current floor for other nodes
        floor_pub_ = create_publisher<std_msgs::msg::String>("/hotel_robot/current_floor", 10);

        // Initial pose publisher (for AMCL re-localization after map switch)
        initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            get_parameter("amcl_initial_pose_topic").as_string(), 10);

        RCLCPP_INFO(get_logger(), "Configuration complete. Current floor: %s", current_floor_.c_str());
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Activating MultiFloorNavigator ...");

        floor_pub_->on_activate();
        initial_pose_pub_->on_activate();

        // Nav2 navigate-to-pose action client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            shared_from_this(),
            get_parameter("nav2_navigate_action").as_string());

        // Elevator service client
        elevator_client_ = create_client<CallElevator>(
            get_parameter("elevator_service").as_string());

        // Map load service client
        map_load_client_ = create_client<LoadMap>(
            get_parameter("map_load_service").as_string());

        // LuggageDelivery action server
        action_server_ = rclcpp_action::create_server<LuggageDelivery>(
            shared_from_this(),
            "/hotel_robot/luggage_delivery",
            std::bind(&MultiFloorNavigator::handle_goal,   this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiFloorNavigator::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiFloorNavigator::handle_accepted, this, std::placeholders::_1)
        );

        publish_current_floor();
        RCLCPP_INFO(get_logger(), "MultiFloorNavigator active.");
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override
    {
        floor_pub_->on_deactivate();
        initial_pose_pub_->on_deactivate();
        action_server_.reset();
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override
    {
        if (db_) { sqlite3_close(db_); db_ = nullptr; }
        persons_sub_.reset();
        floor_pub_.reset();
        initial_pose_pub_.reset();
        nav_client_.reset();
        elevator_client_.reset();
        map_load_client_.reset();
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override
    {
        if (db_) { sqlite3_close(db_); db_ = nullptr; }
        return CallbackReturn::SUCCESS;
    }

private:
    // ─── State ───────────────────────────────────────────────────────────────
    std::string current_floor_;
    std::string db_path_;
    sqlite3*    db_  = nullptr;

    std::mutex persons_mutex_;
    DetectedPersonArray latest_persons_;
    std::atomic<bool>   delivery_running_{false};

    // ─── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp::Subscription<DetectedPersonArray>::SharedPtr persons_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr floor_pub_;
    rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    rclcpp_action::Server<LuggageDelivery>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr  nav_client_;
    rclcpp::Client<CallElevator>::SharedPtr           elevator_client_;
    rclcpp::Client<LoadMap>::SharedPtr                map_load_client_;

    // ─── SQLite helpers ───────────────────────────────────────────────────────
    bool open_database()
    {
        int rc = sqlite3_open(db_path_.c_str(), &db_);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(get_logger(), "SQLite open error: %s", sqlite3_errmsg(db_));
            return false;
        }
        // Ensure schema exists
        const char* create_sql =
            "CREATE TABLE IF NOT EXISTS wormholes ("
            "  id            INTEGER PRIMARY KEY AUTOINCREMENT,"
            "  from_floor    TEXT NOT NULL,"
            "  to_floor      TEXT NOT NULL,"
            "  approach_x    REAL NOT NULL,"
            "  approach_y    REAL NOT NULL,"
            "  approach_yaw  REAL NOT NULL DEFAULT 0.0,"
            "  elevator_id   TEXT NOT NULL,"
            "  map_yaml_path TEXT NOT NULL"
            ");";
        char* err_msg = nullptr;
        sqlite3_exec(db_, create_sql, nullptr, nullptr, &err_msg);
        if (err_msg) {
            RCLCPP_WARN(get_logger(), "Schema init warning: %s", err_msg);
            sqlite3_free(err_msg);
        }
        return true;
    }

    std::vector<Wormhole> query_wormholes(const std::string& from, const std::string& to)
    {
        std::vector<Wormhole> result;
        if (!db_) return result;

        sqlite3_stmt* stmt;
        const char* sql =
            "SELECT from_floor, to_floor, approach_x, approach_y, approach_yaw, "
            "       elevator_id, map_yaml_path "
            "FROM wormholes WHERE from_floor=? AND to_floor=?;";

        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            RCLCPP_ERROR(get_logger(), "Prepare failed: %s", sqlite3_errmsg(db_));
            return result;
        }
        sqlite3_bind_text(stmt, 1, from.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, to.c_str(),   -1, SQLITE_STATIC);

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            Wormhole w;
            w.from_floor    = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            w.to_floor      = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            w.approach_x    = sqlite3_column_double(stmt, 2);
            w.approach_y    = sqlite3_column_double(stmt, 3);
            w.approach_yaw  = sqlite3_column_double(stmt, 4);
            w.elevator_id   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
            w.map_yaml_path = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
            result.push_back(w);
        }
        sqlite3_finalize(stmt);
        return result;
    }

    // ─── Action server callbacks ──────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const LuggageDelivery::Goal> goal)
    {
        RCLCPP_INFO(get_logger(),
            "New delivery request [%s]: %s(%g,%g) → %s(%g,%g) | Guest: %s room %s",
            goal->task_id.c_str(),
            goal->pickup_floor.c_str(),  goal->pickup_x,  goal->pickup_y,
            goal->dropoff_floor.c_str(), goal->dropoff_x, goal->dropoff_y,
            goal->guest_name.c_str(), goal->guest_room.c_str());

        if (delivery_running_) {
            RCLCPP_WARN(get_logger(), "Delivery already in progress – rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLD>)
    {
        RCLCPP_INFO(get_logger(), "Delivery cancel requested.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLD> goal_handle)
    {
        // Run delivery in a separate thread to avoid blocking the executor
        std::thread([this, goal_handle]() { execute_delivery(goal_handle); }).detach();
    }

    // ─── Main delivery execution ──────────────────────────────────────────────
    void execute_delivery(const std::shared_ptr<GoalHandleLD> goal_handle)
    {
        delivery_running_ = true;
        auto start_time = now();
        const auto goal = goal_handle->get_goal();
        const bool direct_room_request =
            goal->pickup_floor == goal->dropoff_floor &&
            std::fabs(goal->pickup_x - goal->dropoff_x) < 1e-3 &&
            std::fabs(goal->pickup_y - goal->dropoff_y) < 1e-3 &&
            std::fabs(goal->pickup_yaw - goal->dropoff_yaw) < 1e-3;
        auto feedback   = std::make_shared<LuggageDelivery::Feedback>();
        auto result     = std::make_shared<LuggageDelivery::Result>();
        uint8_t floors_traversed = 0;

        auto send_feedback = [&](const std::string& phase, float progress) {
            feedback->phase            = phase;
            feedback->current_floor    = current_floor_;
            feedback->progress_percent = progress;
            {
                std::lock_guard<std::mutex> lk(persons_mutex_);
                feedback->persons_detected_nearby =
                    count_nearby_persons(0.0, 0.0,  // rough – filled below
                        get_parameter("person_safety_radius_m").as_double());
            }
            goal_handle->publish_feedback(feedback);
        };

        if (direct_room_request) {
            if (!cross_floors_and_navigate(
                    goal_handle, goal->dropoff_floor,
                    goal->dropoff_x, goal->dropoff_y, goal->dropoff_yaw,
                    "NAVIGATING_TO_ROOM", 0.0f, 95.0f,
                    floors_traversed, feedback))
            {
                result->success = false;
                result->message = "Failed to reach requested room";
                goal_handle->abort(result);
                delivery_running_ = false;
                return;
            }
        } else {
            // ── 1. Navigate to pickup ─────────────────────────────────────────
            if (!cross_floors_and_navigate(
                    goal_handle, goal->pickup_floor,
                    goal->pickup_x, goal->pickup_y, goal->pickup_yaw,
                    "NAVIGATING_TO_PICKUP", 0.0f, 40.0f,
                    floors_traversed, feedback))
            {
                result->success = false;
                result->message = "Failed to reach pickup location";
                goal_handle->abort(result);
                delivery_running_ = false;
                return;
            }

            // ── 2. Wait for luggage to be loaded ─────────────────────────────
            send_feedback("WAITING_FOR_LUGGAGE", 45.0f);
            RCLCPP_INFO(get_logger(), "Waiting 5s for luggage loading...");
            rclcpp::sleep_for(5s);

            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Delivery cancelled during luggage loading";
                goal_handle->canceled(result);
                delivery_running_ = false;
                return;
            }

            // ── 3. Navigate to drop-off ─────────────────────────────────────
            if (!cross_floors_and_navigate(
                    goal_handle, goal->dropoff_floor,
                    goal->dropoff_x, goal->dropoff_y, goal->dropoff_yaw,
                    "NAVIGATING_TO_DROPOFF", 50.0f, 90.0f,
                    floors_traversed, feedback))
            {
                result->success = false;
                result->message = "Failed to reach drop-off location";
                goal_handle->abort(result);
                delivery_running_ = false;
                return;
            }
        }

        // ── 4. Confirmation wait ──────────────────────────────────────────────
        send_feedback("WAITING_FOR_CONFIRMATION", 95.0f);
        RCLCPP_INFO(get_logger(), "Waiting for guest to take luggage...");
        rclcpp::sleep_for(3s);

        // ── 5. Fill result ────────────────────────────────────────────────────
        auto elapsed = (now() - start_time).seconds();
        result->success          = true;
        result->message          = "Delivery completed successfully";
        result->total_time_sec   = static_cast<float>(elapsed);
        result->floors_traversed = floors_traversed;
        result->final_floor      = current_floor_;

        send_feedback("COMPLETE", 100.0f);
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Delivery [%s] done in %.1f s.", goal->task_id.c_str(), elapsed);
        delivery_running_ = false;
    }

    /**
     * Handle multi-floor navigation from current floor to target_floor.
     * If floors are the same → navigate directly.
     * If floors differ → query wormhole DB, navigate to elevator,
     *   call elevator, switch map, re-localize, continue.
     */
    bool cross_floors_and_navigate(
        const std::shared_ptr<GoalHandleLD>& goal_handle,
        const std::string& target_floor,
        double target_x, double target_y, double target_yaw,
        const std::string& phase_prefix,
        float progress_start, float progress_end,
        uint8_t& floors_traversed,
        std::shared_ptr<LuggageDelivery::Feedback>& feedback)
    {
        // Direct navigation on same floor
        if (current_floor_ == target_floor) {
            feedback->phase = phase_prefix + "_SAME_FLOOR";
            goal_handle->publish_feedback(feedback);
            return navigate_to_pose(
                goal_handle, target_x, target_y, target_yaw,
                progress_start, progress_end, feedback);
        }

        // Need to cross floors via elevator (wormhole)
        auto wormholes = query_wormholes(current_floor_, target_floor);
        if (wormholes.empty()) {
            RCLCPP_ERROR(get_logger(),
                "No wormhole from '%s' to '%s' in database",
                current_floor_.c_str(), target_floor.c_str());
            return false;
        }
        const Wormhole& w = wormholes[0];  // take first available elevator

        float mid = (progress_start + progress_end) / 2.0f;

        // 3a. Navigate to elevator approach point
        RCLCPP_INFO(get_logger(),
            "Navigating to elevator '%s' approach (%.2f, %.2f)",
            w.elevator_id.c_str(), w.approach_x, w.approach_y);
        feedback->phase = "NAVIGATING_TO_ELEVATOR";
        feedback->elevator_status = "approaching";
        goal_handle->publish_feedback(feedback);

        if (!navigate_to_pose(goal_handle,
                w.approach_x, w.approach_y, w.approach_yaw,
                progress_start, mid, feedback)) {
            return false;
        }

        // 3b. Call elevator
        RCLCPP_INFO(get_logger(), "Calling elevator '%s' to go to %s",
            w.elevator_id.c_str(), target_floor.c_str());
        feedback->phase = "WAITING_FOR_ELEVATOR";
        feedback->elevator_status = "called";
        goal_handle->publish_feedback(feedback);

        if (!call_elevator(w.elevator_id, current_floor_, target_floor)) {
            RCLCPP_ERROR(get_logger(), "Elevator call failed");
            return false;
        }

        // 3c. Enter elevator and wait during transit
        feedback->phase = "IN_ELEVATOR";
        feedback->elevator_status = "in_transit";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "In elevator – waiting for transit...");
        rclcpp::sleep_for(5s);   // simulated elevator travel time

        // 3d. Switch map to target floor
        if (!switch_floor(target_floor, w.map_yaml_path)) {
            RCLCPP_ERROR(get_logger(), "Map switch to %s failed", target_floor.c_str());
            return false;
        }
        floors_traversed++;

        // 3e. Navigate to final target on new floor
        feedback->phase = phase_prefix + "_NEW_FLOOR";
        feedback->elevator_status = "arrived";
        goal_handle->publish_feedback(feedback);

        return navigate_to_pose(goal_handle,
            target_x, target_y, target_yaw,
            mid, progress_end, feedback);
    }

    // ─── Nav2 NavigateToPose ──────────────────────────────────────────────────
    bool navigate_to_pose(
        const std::shared_ptr<GoalHandleLD>& goal_handle,
        double x, double y, double yaw,
        float progress_start, float progress_end,
        std::shared_ptr<LuggageDelivery::Feedback>& feedback)
    {
        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
            return false;
        }

        // Build Nav2 goal
        auto nav_goal = NavigateToPose::Goal();
        nav_goal.pose.header.stamp    = now();
        nav_goal.pose.header.frame_id = "map";
        nav_goal.pose.pose.position.x = x;
        nav_goal.pose.pose.position.y = y;
        nav_goal.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        nav_goal.pose.pose.orientation = tf2::toMsg(q);

        // Safety: pause if a person is too close
        wait_for_persons_to_clear(goal_handle);

        // Send to Nav2
        auto send_goal_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        bool goal_accepted = false;
        bool nav_done      = false;
        bool nav_success   = false;
        std::mutex done_mutex;
        std::condition_variable done_cv;

        send_goal_opts.goal_response_callback =
            [&](const rclcpp_action::Client<NavigateToPose>::GoalHandle::SharedPtr& handle) {
                goal_accepted = (handle != nullptr);
            };

        send_goal_opts.feedback_callback =
            [&](rclcpp_action::Client<NavigateToPose>::GoalHandle::SharedPtr,
                const std::shared_ptr<const NavigateToPose::Feedback> nav_fb)
            {
                feedback->distance_to_next_waypoint =
                    static_cast<float>(nav_fb->distance_remaining);
                feedback->current_x = x;
                feedback->current_y = y;
                {
                    std::lock_guard<std::mutex> lk(persons_mutex_);
                    feedback->persons_detected_nearby =
                        count_nearby_persons(x, y,
                            get_parameter("person_safety_radius_m").as_double());
                }
                // Interpolate progress from remaining distance
                double max_dist = std::hypot(x, y);  // rough estimate
                if (max_dist > 0.0) {
                    float travelled_frac = std::max(0.0f, std::min(1.0f,
                        1.0f - static_cast<float>(nav_fb->distance_remaining / max_dist)));
                    feedback->progress_percent = progress_start +
                        travelled_frac * (progress_end - progress_start);
                }
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(get_logger(), "Delivery cancelled during navigation.");
                }
                goal_handle->publish_feedback(feedback);

                // Pause if person too close
                wait_for_persons_to_clear(goal_handle);
            };

        send_goal_opts.result_callback =
            [&](const rclcpp_action::Client<NavigateToPose>::GoalHandle::WrappedResult& res) {
                nav_success = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
                {
                    std::lock_guard<std::mutex> lk(done_mutex);
                    nav_done = true;
                }
                done_cv.notify_one();
            };

        nav_client_->async_send_goal(nav_goal, send_goal_opts);

        // Wait for navigation to finish
        std::unique_lock<std::mutex> lk(done_mutex);
        done_cv.wait(lk, [&]{ return nav_done || goal_handle->is_canceling(); });

        if (goal_handle->is_canceling()) return false;
        return nav_success;
    }

    // ─── Elevator service call ────────────────────────────────────────────────
    bool call_elevator(const std::string& elevator_id,
                       const std::string& from, const std::string& to)
    {
        if (!elevator_client_->wait_for_service(5s)) {
            RCLCPP_WARN(get_logger(), "Elevator service not available – simulating");
            rclcpp::sleep_for(3s);
            return true;  // simulate success in absence of real elevator hardware
        }
        auto req = std::make_shared<CallElevator::Request>();
        req->current_floor = from;
        req->target_floor  = to;
        // Elevator ID is embedded in the request via a naming convention
        // (real systems may use a different field or topic)
        (void)elevator_id;

        auto future = elevator_client_->async_send_request(req);
        if (future.wait_for(30s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Elevator service timeout");
            return false;
        }
        auto resp = future.get();
        RCLCPP_INFO(get_logger(), "Elevator response: %s (wait %.1fs)",
            resp->message.c_str(), resp->wait_time_sec);
        return resp->success;
    }

    // ─── Map / floor switching ────────────────────────────────────────────────
    bool switch_floor(const std::string& new_floor, const std::string& map_yaml)
    {
        RCLCPP_INFO(get_logger(), "Switching to floor '%s' (map: %s)",
            new_floor.c_str(), map_yaml.c_str());

        if (!map_load_client_->wait_for_service(5s)) {
            RCLCPP_WARN(get_logger(),
                "Map load service unavailable – assuming simulation mode, floor switched");
            current_floor_ = new_floor;
            publish_current_floor();
            return true;
        }

        auto req = std::make_shared<LoadMap::Request>();
        req->map_url = map_yaml;

        auto future = map_load_client_->async_send_request(req);
        if (future.wait_for(10s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "LoadMap service timeout");
            return false;
        }

        auto resp = future.get();
        if (resp->result != LoadMap::Response::RESULT_SUCCESS) {
            RCLCPP_ERROR(get_logger(), "LoadMap failed with result code %d", resp->result);
            return false;
        }

        current_floor_ = new_floor;
        publish_current_floor();

        // Re-initialize AMCL pose at elevator exit position
        re_localize_at_elevator_exit(new_floor);

        // Brief wait for costmap to update
        rclcpp::sleep_for(2s);
        return true;
    }

    void re_localize_at_elevator_exit(const std::string& floor)
    {
        (void)floor;

        // Keep AMCL aligned with the Gazebo teleport pose after elevator travel.
        // The DB approach yaw is for entering the elevator, not for the exit pose.
        const double exit_x = 4.80;
        const double exit_y = 0.0;
        const double exit_yaw = M_PI;

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp    = now();
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = exit_x;
        pose.pose.pose.position.y = exit_y;
        tf2::Quaternion q; q.setRPY(0, 0, exit_yaw);
        pose.pose.pose.orientation = tf2::toMsg(q);
        pose.pose.covariance[0]  = 0.05;
        pose.pose.covariance[7]  = 0.05;
        pose.pose.covariance[35] = 0.05;

        for (int i = 0; i < 5; ++i) {
            pose.header.stamp = now();
            initial_pose_pub_->publish(pose);
            rclcpp::sleep_for(150ms);
        }

        RCLCPP_INFO(get_logger(),
            "Published AMCL elevator exit pose: (%.2f, %.2f, %.2f rad)",
            exit_x, exit_y, exit_yaw);
    }

    // ─── Person safety helpers ────────────────────────────────────────────────
    uint32_t count_nearby_persons(double ref_x, double ref_y, double radius)
    {
        uint32_t count = 0;
        for (const auto& p : latest_persons_.persons) {
            double dx = p.position.x - ref_x;
            double dy = p.position.y - ref_y;
            if (std::hypot(dx, dy) < radius) count++;
        }
        return count;
    }

    void wait_for_persons_to_clear(const std::shared_ptr<GoalHandleLD>& goal_handle)
    {
        double safety_r = get_parameter("person_safety_radius_m").as_double();
        double timeout  = get_parameter("person_pause_timeout_sec").as_double();
        auto   deadline = now() + rclcpp::Duration::from_seconds(timeout);

        bool warned = false;
        while (rclcpp::ok() && !goal_handle->is_canceling()) {
            std::lock_guard<std::mutex> lk(persons_mutex_);
            // Use robot origin as ref (simplified – real impl would use TF)
            uint32_t near = count_nearby_persons(0.0, 0.0, safety_r);
            if (near == 0) break;

            if (!warned) {
                RCLCPP_WARN(get_logger(),
                    "Person within %.1f m safety radius – pausing navigation", safety_r);
                warned = true;
            }
            if (now() > deadline) {
                RCLCPP_WARN(get_logger(),
                    "Person timeout exceeded – resuming anyway (slow mode)");
                break;
            }
            rclcpp::sleep_for(500ms);
        }
    }

    // ─── Misc ─────────────────────────────────────────────────────────────────
    void publish_current_floor()
    {
        std_msgs::msg::String msg;
        msg.data = current_floor_;
        floor_pub_->publish(msg);
    }
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 4);
    auto node = std::make_shared<MultiFloorNavigator>();
    executor->add_node(node->get_node_base_interface());

    // Automatically bring node through lifecycle for convenience
    node->configure();
    node->activate();

    RCLCPP_INFO(node->get_logger(), "MultiFloorNavigator running.");
    executor->spin();
    rclcpp::shutdown();
    return 0;
}
