# sketch2nav

> Draw your own obstacle course and see it come to life.

A ROS2 robotics project that lets you sketch navigation paths and obstacles on a web canvas and have a rover autonomously follow them in Gazebo simulation or soon come to real-world deployment.
---
## Overview

sketch2nav replaces traditional waypoint-entry interfaces with a freehand drawing tool. The user sketches a path on a browser canvas published as a `nav_msgs/Path` over WebSocket via rosbridge, and executed on a simulated skid-steer rover using a different controllers implemented from scratch.

**Key technical contributions:**

- Custom browser-based path input interface communicating with ROS2 via rosbridge WebSocket
- Catmull-Rom spline interpolation for smooth path generation from raw mouse input
- Currently Pure Pursuit path-following controller with circle-segment intersection lookahead
- Real-time parameter tuning (lookahead distance, max speed) via ROS2 parameter service from the UI
- Pixel-to-world coordinate transform with configurable scale factor
- Gazebo Harmonic simulation with full ROS2 bridge for `cmd_vel` and `odom`
---
## Demo
Coming Soon
