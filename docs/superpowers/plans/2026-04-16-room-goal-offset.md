# Room Goal Offset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all room arrival goals slightly closer to the room doors.

**Architecture:** Keep the change localized to the GUI room target table so the navigator, elevator logic, and room-selection flow remain unchanged. Apply a small symmetric offset to all room Y coordinates, preserving room IDs and yaw values.

**Tech Stack:** Python 3.10, ROS2 Humble, Tkinter

---

### Task 1: Adjust Room Targets

**Files:**
- Modify: `/home/yusuf/bitirme/hotel_luggage_robot/scripts/hotel_gui.py`

- [ ] Change the shared room target Y coordinates from `-1.8` / `1.8` to slightly closer door-approach values.
- [ ] Keep all room IDs, floors, and yaw values unchanged.

### Task 2: Verify

**Files:**
- Test: `/home/yusuf/bitirme/hotel_luggage_robot/scripts/hotel_gui.py`

- [ ] Run `python3 -m py_compile /home/yusuf/bitirme/hotel_luggage_robot/scripts/hotel_gui.py`.
- [ ] Relaunch the stack and confirm room goals now stop closer to the doors.
