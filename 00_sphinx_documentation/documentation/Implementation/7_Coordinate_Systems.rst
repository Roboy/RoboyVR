Coordinate Systems
==================

General
-------
Coordinate systems present an immense hazard when implementing messages from Unity to Gazebo or back. In case these messages contain positions, rotations and directions, it is of **utmost improtance** to transform coordinates into the **expected coordinate spaces**: 

- Unity's coordinate system: 

 - The x coordinate points to the right (right edge of the screen)
 - The y coordinate points upwards (top of the screen)
 - The z coordinate points into the scene, away from the user

- Gazebo's coordinate system:

 - The x coordinate points to the right (right edge of the screen)
 - The y coordinate points into the scene, away from the user
 - The z coordinate points upwards (top of the screen)

**NOTICE: y and z coordinate are swapped when converting coordinates into the other coordinate system**

**Implementation**

A utility class called *GazeboUtility* provides conversions for both angles and rotations from and to Gazebo coordinates. All defined messages automatically transform the values, depending on if they were parsed from a received message or created using the Unity constructors. 

World and local space
---------------------
Another important aspect to consider: Positions and forces can be defined in world space, giving the absolute position / direction, or they can be defined in local space. When values are defined in local space, the local coordinate system always needs to be provided as well. 

**Message examples**
The *ExternalForceMessage* is an exellent example: A force is **defined locally**, with its relative position and relative direction specified **along with the Roboy part** . The part provides the local coordinate system. 
Other messages may specify positions locally or in worls space, when in doubt: The message definition *should* provide further information, otherwise check the receiving entity how it handles the values. 

**Simulations in Gazebo**
In gazebo, every position can be queried in local or world coordinate space, and forces can be applied locally (*AddRelativeForce(...)*) or in world space (*AddWorldForce(...)*). 

**Issues**
When applying relative forces in Gazebo, the results never matched the given forces and directions. One option, how to solve this issue, would be to convert the local values to world coordinate space. Unfortunately, during the given time, no function was found which transforms the values correctly (The scale of the coordinate systems might be one of the reasons it failed). 

For now, external forces are defined **in World Space** and the respective gazebo simulation applies forces in world space as well. 