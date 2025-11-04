# niryo-ros-project  
Gesture control on Niryo using OpenCV.

---

## Ideal Path Structure
```bash
gesture_control_niryo_ws/
├── src/
│   ├── gesture_recognition/ `actually completed`
│   │   ├── gesture_recognition/
│   │   │   ├── __init__.py
│   │   │   ├── gesture_detector.py          # OpenCV gesture detection node
│   │   │   ├── gesture_classifier.py        # Classify gestures (wave, thumbs up/down)
│   │   │   └── camera_handler.py            # Camera interface
│   │   ├── config/
│   │   │   ├── camera_params.yaml           # Camera settings
│   │   │   └── gesture_params.yaml          # Detection thresholds
│   │   ├── launch/
│   │   │   └── gesture_detection.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── gesture_control_interface/
│   │   ├── gesture_control_interface/
│   │   │   ├── __init__.py
│   │   │   ├── gesture_to_command.py        # Map gestures to robot commands
│   │   │   └── command_publisher.py         # Publish commands to robot
│   │   ├── msg/
│   │   │   ├── Gesture.msg                  # Custom gesture message
│   │   │   └── RobotCommand.msg             # Custom command message
│   │   ├── srv/
│   │   │   └── SetGestureMode.srv           # Service to change control mode
│   │   ├── config/
│   │   │   └── gesture_mappings.yaml        # Gesture-to-action mappings
│   │   ├── launch/
│   │   │   └── gesture_interface.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── CMakeLists.txt
│   │
│   ├── niryo_control/
│   │   ├── niryo_control/
│   │   │   ├── __init__.py
│   │   │   ├── robot_controller.py          # Abstract robot controller
│   │   │   ├── real_robot_controller.py     # Real Niryo Ned2 interface
│   │   │   └── sim_robot_controller.py      # Gazebo simulation interface
│   │   ├── config/
│   │   │   ├── real_robot.yaml              # Real robot parameters
│   │   │   └── sim_robot.yaml               # Simulation parameters
│   │   ├── launch/
│   │   │   ├── real_robot.launch.py         # Launch for real robot
│   │   │   ├── sim_robot.launch.py          # Launch for simulation
│   │   │   └── gazebo_world.launch.py       # Gazebo environment
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── niryo_gazebo/
│   │   ├── worlds/
│   │   │   └── niryo_workspace.world        # Gazebo world file
│   │   ├── models/
│   │   │   └── niryo_ned2/                  # Robot URDF/meshes (if not using official)
│   │   ├── launch/
│   │   │   └── spawn_niryo.launch.py
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── gesture_control_bringup/
│       ├── launch/
│       │   ├── full_system_real.launch.py   # Launch everything for real robot
│       │   ├── full_system_sim.launch.py    # Launch everything for simulation
│       │   └── demo.launch.py               # Demo with pre-recorded gestures
│       ├── config/
│       │   └── system_params.yaml           # Global parameters
│       ├── package.xml
│       └── CMakeLists.txt
```

## Key Design Decisions
1. Modular Package Structure
gesture_recognition → Pure vision processing, no robot knowledge
gesture_control_interface → Translates gestures to robot-agnostic commands
niryo_control → Robot-specific implementation with abstraction for real/sim
niryo_gazebo → Simulation assets
gesture_control_bringup → System integration and launch files

2. Message Flow
`Camera → gesture_detector → Gesture.msg → gesture_to_command → RobotCommand.msg → robot_controller → Niryo (real/sim)`

3. Suggested Gesture Mappings
Gesture	Action
Wave	Home position / Reset
Thumbs Up	Execute predefined action / Confirm
Thumbs Down	Stop / Cancel operation

