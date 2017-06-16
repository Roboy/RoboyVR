Developer's Manual
==================

*Note: We assume that you already have gone through the User's Manual to not repeat ourselves.*

Gazebo Plugin
-------------

The main part on the Gazebos site is the plugin *ForceJointPlugin*. The location is:

.. code:: bash

  path-to-roboy-ros-control/src/roboy_simulation/src/ForceJointPlugin.cpp

The plugin does the following. 

1) It loads the model into Gazebo.
2) It starts a topic with type Float32 as message type for every revolute joint of the PaBi model.
3) It subscribes to the created topics.
4) It creates a publisher which publishes the pose of PaBi so we can subscribe to the topic on the Unity side.
5) It makes PaBi stationary so he does not fall down when the legs are not touching the ground.

The topic names have a special structure:

.. code:: bash

  /roboy/pabi_angle/<joint_name>

At the current state of the model there are four topics with Float32 message type:

.. code:: bash

  /roboy/pabi_angle/hip_1
  /roboy/pabi_angle/hip_2
  /roboy/pabi_angle/knee_1
  /roboy/pabi_angle/knee_2

The pose is published with message type *roboy_communication_simulation::Pose* the topic:

.. code:: bash

  /roboy/pabi_pose

If you want PaBi to be able to fall down change the *OnUpdate* method in the plugin.

The main functions of the plugin are:

1) Load: It loads the model into gazebo and creates the joint subscribers and the pose publisher.
2) OnRosMsg: Is called every time the plugin receives a joint message. It updates the joint angles value list and publishes the new state.
3) publishPose: Publishes the pose of PaBi.
4) OnUpdate: Is called every gazebo update frame. Therefore we have to zero out the forces of PaBi and update the joint angles of the
actual model.

*Note: Change the following line in OnUpdate if you want PaBi to be able to fall down:

.. code:: bash

  model->SetWorldPose(initPose);

PaBiDanceSimulatorNode
----------------------

This ROS node creates four publishers for the joints of PaBi. In the Main loop it publishes new joint angles.
To make the movement smooth the published joint angles are changed gradually in small steps from -90° to 0° and back.
Therefore we have two functions. One to start the animation:

.. code:: c++

  void PaBiDanceSimulator::startDanceAnimation()
  {
      while(ros::ok())
      {
	  if(adjustPoseGradually(true))
	      adjustPoseGradually(false);
      }
  }

And another to adjust the pose:

.. code:: c++

  bool PaBiDanceSimulator::adjustPoseGradually(bool goUp)
  {
      float stepSize = 1;
      int sleeptime = 10000;
      // adjusts the joint angles to -90° in 90 * stepSize * 0.01 seconds
      if(goUp)
      {
          float currentAngle = 0;
          while(currentAngle > -90)
          {
              publishAngles(currentAngle);
              usleep(sleeptime);
              currentAngle -= stepSize;
          }
      }
      else
      {
          float currentAngle = -90;
          while(currentAngle < 0)
          {
              publishAngles(currentAngle);
              usleep(sleeptime);
              currentAngle += stepSize;
          }
      }
      return true;
  }

Unity Scene
-----------

In Unity we have the ROSBridge which connects to the ROSBridge on the simulation side. On the PaBi legs we have a ROSObject attached

