# Near-Field Person Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep moving people detectable and avoidable when they are very close to the robot.

**Architecture:** Tighten near-range camera-LiDAR fusion in `person_detector.py` and preserve close tracks briefly with motion prediction so costmap updates do not disappear when a person fills the camera view. Add a few detector parameters in `detection.yaml` to tune close-range behavior without touching Nav2 logic.

**Tech Stack:** ROS2 Humble, Python 3.10, Nav2, YOLOv8, NumPy

---

### Task 1: Add Near-Field Detector Parameters

**Files:**
- Modify: `hotel_luggage_robot/config/detection.yaml`
- Modify: `hotel_luggage_robot/scripts/person_detector.py`

- [ ] Add close-range parameters for fusion margin, track persistence, and synthetic scan widening.
- [ ] Load and cache the new parameters in `PersonDetectorNode.__init__`.

### Task 2: Improve Near-Field Fusion

**Files:**
- Modify: `hotel_luggage_robot/scripts/person_detector.py`

- [ ] Expand bbox matching when the person is close or the bbox is large.
- [ ] Prefer robust nearby projected LiDAR hits before falling back to monocular projection.
- [ ] Preserve a useful `dist` estimate for close detections so synthetic obstacle generation can react immediately.

### Task 3: Make Close Tracks Persist Briefly

**Files:**
- Modify: `hotel_luggage_robot/scripts/person_detector.py`

- [ ] Add short predictive persistence for close tracks so moving people do not disappear when they momentarily leave the bbox-LiDAR overlap.
- [ ] Decay stale close tracks quickly enough to remain dynamic.

### Task 4: Strengthen Synthetic Close-Range Obstacle Scan

**Files:**
- Modify: `hotel_luggage_robot/scripts/person_detector.py`

- [ ] Widen the synthetic person scan cone for near people.
- [ ] Increase close-range obstacle density so Nav2 replans immediately instead of waiting for a new far-field fusion hit.

### Task 5: Verify

**Files:**
- Test: `hotel_luggage_robot/scripts/person_detector.py`

- [ ] Run `python3 -m py_compile hotel_luggage_robot/scripts/person_detector.py`.
- [ ] Run `colcon build --packages-select hotel_luggage_robot --symlink-install --event-handlers console_direct+`.
- [ ] Relaunch the stack and verify `/plan` still publishes while close people remain visible in RViz.
