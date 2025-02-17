# Custom ROS plugin

[Creating and using plugins (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)

[Ros2 Pluginlib](http://wiki.ros.org/pluginlib)

## Commands

1. creating the interface(base): `ros2 pkg create --build-type ament_cmake --license Apache-2.0 --dependencies pluginlib --node-name area_node polygon_base`
2. creating the plugin: `ros2 pkg create --build-type ament_cmake --license Apache-2.0 --dependencies polygon_base pluginlib --library-name polygon_plugins polygon_plugins`