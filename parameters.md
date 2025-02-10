# Parameters in ROS2

ROS2 parameters provide a way to customize the behavior of nodes in a flexible manner. They allow you to modify node configurations without needing to recompile or redeploy code, making them an essential tool for robot behavior configuration.

A **parameter** can be thought of as a runtime setting that can influence how a node behaves or interacts with other components in the system. Parameters are particularly useful for changing a robot's behavior dynamically, without altering the underlying code.

For example, in applications like **MoveIt2**, parameters allow the configuration of different behaviors based on the robot's specific characteristics. This can include choosing whether to plan trajectories for a simple 3-DOF (3 joints and 3 motors) robot or a more complex 6-DOF (6 joints and 6 motors) robot. 

Parameters allow developers to fine-tune the software for specific hardware setups, whether that’s adjusting control algorithms, sensor parameters, or motion planning strategies.

### Key Points About ROS2 Parameters

- **Customizability:** Parameters enable you to adjust node behavior according to different hardware or software setups. For example, controlling robot velocity, sensor thresholds, or specific motion settings.
- **Dynamic Configuration:** You can update parameters at runtime without restarting or redeploying your node.
- **Universal Node Logic, Unique Behavior:** The core logic of a node remains the same across all robots, but parameters let each robot instance behave differently, such as with different sensor configurations, performance tuning, or operational modes.
- **Scalability:** Parameters make it easier to scale software across different robots with varying hardware and sensor configurations, especially in large fleets of robots.
  
### Types of Parameters in ROS2

1. **Static Parameters**  
   These are parameters that do not change after the node starts and are typically set at launch time. Static parameters are often used to configure hardware-specific settings like robot dimensions or initial poses.

2. **Dynamic Parameters**  
   These parameters can be changed while the node is running. This allows for real-time tuning, such as adjusting PID controllers or modifying sensor sensitivities based on environmental factors.

3. **Launch Parameters**  
   These are parameters passed during the node launch and can be set either in the launch file or directly via command-line arguments.

4. **Parameter Types**  
   Parameters can be of various types, such as:
   - **Boolean**
   - **Integer**
   - **Float**
   - **String**
   - **Array**
   - **Nested parameters (dictionaries of other parameters)**

### Modifying Parameters

Parameters can be set or modified using:
- **CLI**: ROS2 provides a command-line tool (`ros2 param`) to get, set, or list parameters of running nodes.
- **Service Calls**: Nodes can expose services for dynamic parameter updates.
- **Launch Files**: Parameters can be defined in launch files, allowing for easier management in large systems.

### Using Parameters

1. **Get Parameter**: You can retrieve the current value of a parameter using the ROS2 CLI or in code.
2. **Set Parameter**: Update a parameter at runtime to change node behavior.
3. **List Parameters**: View the current parameters available to a node.
4. **Callback**: A node can define a callback to react to parameter changes in real-time.

### Example Usage

```bash
# List parameters for a running node
ros2 param list /my_node

# Set a parameter
ros2 param set /my_node max_speed 2.5

# Get a parameter value
ros2 param get /my_node max_speed
```

### Advantages of Using Parameters

- **Modularity:** Allows nodes to be configured independently without changing source code.
- **Flexibility:** Easily tune robot behaviors based on specific tasks or environments.
- **Consistency:** Use the same node with different configurations for various robots, ensuring consistency in node logic.
  
### Conclusion

Parameters in ROS2 allow for flexible, customizable, and scalable robotic software development. They are a key tool for dynamic configuration, enabling robots to adapt their behavior based on their hardware setup and operational environment. By using parameters effectively, developers can achieve better control over robot systems without needing to alter source code for every specific scenario.
