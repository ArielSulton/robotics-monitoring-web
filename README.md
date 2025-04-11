# Web Based Robotics Monitoring System

## Tech Stack
- ROS 2 Foxy
- Flask Python
- HTML, CSS, JS

## Terminal Session Configuration

### Terminal 1: "vehicle/" directory
```
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 run publisher talker
```

### Terminal 2: "website/" directory
```
python3 app.py
```

## Development URL Access
```
http://127.0.0.1:5000
```