# Commands to get you started with ROS2

1. `colcon build` - This will build all the packages within your **workspace/src/**. Should be run on the parent folder containing the the source directory.
2. `ros2 pkg create --build-type <build-tool> <package-name>` e.g.

    ```bash
    ros2 pkg create --build-type ament_python arduinobot_py_examples
    ros2 pkg create --build-type ament_cmake arduinobot_cpp_examples
    ```
3. `ros2 pkg list` - lists all the packages present within your ros2 both within the underlay and the overlay packages.

4. `ros2 run <package-name> <node-name>` - to start a ROS 2 node e.g. `ros2 run arduinobot_py_examples simple_publisher`

5. `ros2 topic list` - lists all the topics available at the moment

6. `ros2 topic echo <topic-name>` - subscribes to a topic and prints on the console the message transmitted on the topic real time e.g. `ros2 topic echo /chatter`

7. `ros2 topic info <topic-name>` - gives you the information about a topic e.g. `ros2 topic info /chatter`. You can add other flags like `--verbose` or `-v` to get even more information about the topic. You can get information such as the type of message the topic prints, the node publishing the topic, number of nodes subscribed to the topic etc.

8. `ros2 topic hz <topic-name>` - analyzes the messages printed on the topic and calculates the frequency at which they are published.

9. `ros2 topic pub <topic-name> <msg-type> <msg-content>` - publishes a message to a topic e.g. `ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS 2'"`

# `package.xml`
In ROS 2, the `package.xml` file contains several tags to specify different types of dependencies for a package. Here’s a list of the common `_depend` tags along with their usage:

## Dependency Tags in `package.xml`

1. **`<build_depend>`**
   - **Usage**: This tag is used to specify packages that are required **only during the build process**. If your package needs another package to compile or build but does not require it at runtime, you should use this tag.
   - **Example**: 
     ```xml
     <build_depend>some_build_dependency</build_depend>
     ```

2. **`<exec_depend>`**
   - **Usage**: This tag is for packages that are needed **only at runtime**. If your package utilizes another package's functionality while it runs, list it here.
   - **Example**: 
     ```xml
     <exec_depend>some_runtime_dependency</exec_depend>
     ```

3. **`<test_depend>`**
   - **Usage**: This tag specifies dependencies that are required **only for testing** your package. If you have libraries or tools that are only needed for running tests, use this tag.
   - **Example**: 
     ```xml
     <test_depend>some_test_dependency</test_depend>
     ```

4. **`<build_export_depend>`**
   - **Usage**: This tag is used for dependencies that are needed during the build process and must also be available to other packages that depend on this package. It is typically used when exporting headers or libraries.
   - **Example**: 
     ```xml
     <build_export_depend>some_export_dependency</build_export_depend>
     ```

5. **`<depend>`**
   - **Usage**: This is a catch-all tag that indicates a dependency that is needed for building, exporting, and executing the package. Use this when a dependency falls into multiple categories and you want to simplify your `package.xml`.
   - **Example**: 
     ```xml
     <depend>some_general_dependency</depend>
     ```

### Summary of When to Use Each Tag:
- Use `<build_depend>` for dependencies required only during compilation.
- Use `<exec_depend>` for dependencies needed only at runtime.
- Use `<test_depend>` for dependencies needed solely for testing purposes.
- Use `<build_export_depend>` for dependencies necessary during building and also required by other packages that depend on yours.
- Use `<depend>` when a dependency is required across multiple phases (build, export, execute).

This structured approach helps in managing dependencies effectively within ROS 2 packages, ensuring that each dependency is categorized according to its role in the package lifecycle.

## [Learn how you can use CMakeListst.txt](https://ros2docs.robook.org/humble/How-To-Guides/Ament-CMake-Documentation.html)