para probar los nodos:

	terminal 1:
		source /opt/ros/jazzy/setup.bash
		colcon build --packages-select viz_package_cpp2
		source install/setup.bash
		ros2 run viz_package_cpp2 wheel_subscriber
		
	terminal 2:
		source /opt/ros/jazzy/setup.bash
		source install/setup.bash
		ros2 run viz_package_cpp wheel_publisher

	terminal 3:
		source /opt/ros/jazzy/setup.bash
		source install/setup.bash
		ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

a la altura donde está el src, build, intall, log
