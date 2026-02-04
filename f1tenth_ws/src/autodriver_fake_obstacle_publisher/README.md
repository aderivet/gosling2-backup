sudo apt install ros-${ROS_DISTRO}-derived-object-msgs

ros2 run autodriver_fake_obstacle_publisher fake_obstacle_publisher --ros-args -p config_file:="src/autodriver_fake_obstacle_publisher/config/obstacles_config.yaml"