# Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is used in ROS2 to represent the structure and components of a robot in XML format. It defines how different parts of the robot are organized, how they are connected, and their properties. URDF is essential for simulating robot behavior, particularly when using a physics engine to model interactions with the environment.

## Key Components of URDF

The URDF consists of several XML tags that define the robot’s components and their relationships. Below is an example illustrating the main URDF tags:

```xml
<robot>
    <link>
        <name></name>
        <visual></visual>
        <collision></collision>
        <inertial></inertial>
    </link>

    <joint>
        <type></type>
        <parent></parent>
        <child></child>
        <origin></origin>
        <limit></limit>
        <axis></axis>
    </joint>
</robot>
```

### 1. `<robot>`
- This is the root element that defines the structure of the robot. It acts as a container for all the components (links and joints) and specifies how they are connected within the robot model.

### 2. `<link>`
- A link represents a component or part of the robot, such as a limb, body part, or sensor. It creates a reference frame and can include several sub-tags to define its properties:
  - **`<name>`**: A unique identifier for the link.
  - **`<visual>`**: Specifies the appearance of the link, including a 3D model or mesh, which is useful for visualization purposes.
  - **`<collision>`**: Defines the physical volume of the link, typically used for collision detection in simulations.
  - **`<inertial>`**: Defines the inertia properties of the link (mass, center of mass, and moments of inertia), important for physics simulations.
  
These properties allow the physics engine to simulate how the robot behaves and interacts with its environment.

### 3. `<joint>`
- A joint connects two links and defines the relationship between them. Joints allow for movement or fixed connections between parts of the robot. The joint element includes:
  - **`<parent>`** and **`<child>`**: Define which two links are connected. The parent link is the reference, and the child link is attached to it.
  - **`<type>`**: Specifies the type of joint, e.g., fixed, revolute (rotational), prismatic (translational), etc. This defines how the two connected links move relative to each other.
  - **`<origin>`**: Defines the position and orientation of the joint in relation to the parent link.
  - **`<limit>`**: Specifies limits on the relative movement of the links connected by the joint (e.g., maximum angle or distance).
  - **`<axis>`**: Defines the axis of rotation or translation for the joint (for revolute or prismatic joints).

The joint structure follows a tree-like hierarchy:
- Each link can have one parent link, but a parent can have multiple child links connected to it. This creates the robot's kinematic chain.

## How URDF Works

By reusing these tags for different components (links and joints), and specifying their properties and relationships, you can build a complete model of your robot. This model can then be used for tasks like simulation, control, and visualization. URDF provides the foundation for describing both simple and complex robots, from stationary arms to mobile robots with multiple degrees of freedom.
