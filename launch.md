# Launch Files in ROS2

Launch files in ROS2 provide a way to organize and automate the startup of multiple nodes and processes, streamlining the management of complex robotic applications. They enable the execution of a list of functionalities with a single command, eliminating the need to manually start each node or process in separate terminal windows.

## Why Launch Files Are Needed

- In typical ROS2 applications, you often need to run multiple commands across different terminal windows. For example, you may start:
  - A **Robot State Publisher** node that reads and publishes the URDF model of the robot.
  - A **Joint State Publisher** node that reads and updates the current state and position of the robot's joints.
  - A **RViz** node to visualize the robot's components.
  
- As applications become more complex, you may need to launch even more nodes and tools to handle things like parameter configuration or log viewing, further complicating the startup process. 

- The number of nodes and the complexity of their interdependencies grow quickly with increasing robot capabilities. As a result, managing multiple terminals, each handling different nodes and commands, becomes inefficient and error-prone.

## How Launch Files Simplify the Process

ROS2 provides **launch files** to automate and simplify the process of starting multiple nodes and processes. These files, written in **Python**, **XML**, or **YAML**, contain a list of operations and instructions to start specific applications or functionalities.

With a single command, `ros2 launch`, you can trigger the execution of all the required operations and nodes. The launch file allows you to:
- **List the nodes** to be started.
- **Declare dependencies** between nodes, ensuring that one starts before another or waits for the completion of another.
- **Configure parameters** for nodes, ensuring they are properly initialized before they start running.
- Start **external processes or applications** (outside of ROS2) that interact with your robot.

### Key Components of a Launch File

Launch files in ROS2 are built using three primary components:
1. **Nodes** – Individual ROS2 nodes that run specific tasks.
2. **Parameters** – Configuration values used by nodes to customize their behavior.
3. **Processes** – External processes or applications (outside of ROS2) that can interact with the robot.

### Example of Managing Node Execution

As complexity increases, you might think about placing all node start-up instructions in a single launch file. However, this violates the principles of **composition** and **reusability**, which are crucial to building scalable and maintainable robotic systems.

Instead, launch files should be structured logically and modularly:
- A launch file should include operations that are functionally related, such as launching all components for a particular robot behavior or functionality.
- Multiple smaller launch files can be created for different parts of the system (e.g., one for motion planning, one for sensors, etc.).
- These individual launch files can then be **stacked** (combined) into a higher-level launch file that starts the entire robot system, respecting dependencies and priorities.

### Advantages of Launch Files

- **Modularity**: You can create separate launch files for different functionalities and combine them as needed.
- **Reusability**: Launch files can be reused for various robots or different configurations of the same robot.
- **Efficiency**: A single command (`ros2 launch`) starts all nodes and processes, making the development process faster and less error-prone.
- **Seamless Integration**: Dependencies and parameters for nodes can be automatically managed without manual intervention.

### Launch File Types and Their Limitations

- **XML** and **YAML** are supported but provide limited functionality compared to Python-based launch files.
  - XML and YAML are more declarative and are used primarily for simple, less flexible configurations.
  - Python-based launch files allow more advanced control, including logic for node startup, conditions, and event handling.

## ROS2 Launch Libraries

To work with launch files in ROS2, two main libraries are used:

1. **`launch`**: Handles the management and configuration of the launch file itself. This includes managing the interface with the user, other launch files, and the operating system.
2. **`launch_ros`**: Contains ROS2-specific functionality, such as managing ROS2 nodes, parameters, and interactions between ROS2 components.

### Key Features in `launch` and `launch_ros`

| `launch`              | `launch_ros`          |
|-----------------------|-----------------------|
| **Actions**           | **Actions**           |
| **Substitutions**     | **Substitutions**     |
| **Event Handlers**    | **Event Handlers**    |
| **Conditions**        | **Parameter Descriptions** |

- **Actions**: These specify what tasks to perform (e.g., launching a node or starting a process).
- **Substitutions**: Used to inject dynamic values into launch files (e.g., setting node names, paths, etc.).
- **Event Handlers**: Allow you to handle runtime events, such as managing timeouts or node failures.
- **Conditions**: Control the execution flow based on conditions like parameters or the state of nodes.
- **Parameter Descriptions**: In `launch_ros`, you can declare parameters for ROS2 nodes and set their values during launch.

## Conclusion

Launch files in ROS2 provide a powerful and flexible way to manage and automate the startup of complex robotic systems. By organizing nodes, parameters, and processes into modular launch files, you can streamline the configuration and execution of different robot functionalities. This not only saves time but also promotes the reusability and maintainability of your ROS2 applications.
