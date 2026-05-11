# wall_follow_node

## Description
`wall_follow_node` is a ROS 2 package containing the node `wall_follow_node`
- **Wall Follow Node**: Subscribes to `scan` for LiDAR measurements. At each scan, it takes one ray to the car's right and another ray (0, 70] degrees above it. These two rays are used to compute the error between the desired distance (1 meter from the right wall) and the actual distance, along with the wall angle using a 1.5 meter lookahead. This error is passed into a PID controller to generate the steering angle and velocity, which are published to `drive`.

## Run
Make sure the package is built in your ROS 2 workspace:
```
cd /racerbot_ws
colcon build --packages-select wall_follow_node
source install/setup.bash
```

1. Run nodes with a launch file:
```
ros2 launch wall_follow_node wall_follow_node_launch.py
```
2. Or run the node individually:
```
ros2 run wall_follow_node wall_follow_node
```

## Parameters
- None

## Topics

### Published
- `drive`

### Subscribed
- `scan`
