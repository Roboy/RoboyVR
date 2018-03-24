.. Software Documentation template master file, created by
   sphinx-quickstart on Fri Jul 29 19:44:29 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to the RoboyVR documentation!
=====================================

.. figure:: images/roboy.*
   :align: center
   :alt: Image could not be loaded
   
What is it?
-----------

RoboyVR is a virtual reality experience in which the user can experience and interact with Roboy (a humanoid robot) in a virtual environment. He or she can watch and study Roboy while he performs specific tasks, e.g. walking, waving, etc., while receiving additional information about the robot's state. The user can also interact with him, move him around, shove or pull him. Other features include BeRoboy, where the user can experience the virtual world through Roboy's eyes, see what he sees and move his body as if it was his or her own.  

How does it work?
-----------------

The virtual environment is run using Unity, a game development platform. In the project, a model of Roboy in a virtual environment is provided and the user interaction is detected and processed here. Roboy and its behavior are simulated in Gazebo on a Linux OS and the relevant information such as positions and forces are shared using ROS, a tool for inter-platform communication. Forces or input from the user are sent from Unity to Gazebo, where these values are used to simulate Roboy in a physically accurate way. The resulting new positions are then provided back to Unity, which updates the positions of the displayed model. Additional information which is provided, such as forces applied to tendons, angles and notifications, are displayed in a custom User Interface. 

Current status of the project
-----------------------------

Currently, Unity provides a model which adapts its position according to the values provided by a simulation. It can display additional information such as tendons and its forces as well as notifications when provided with these. Motor values are stored and processed, but not visualised as of now. 

Older code previously implemented: 
BeRoboy enables the user to see through Roboy's eyes, the head and hand position of him or her are mapped to the model to move along with the user. Furthermore, different Roboy models can be downloaded from an online repository and will automatically be imported into Unity. A selection of these can be spawned in the virtual environment and later be removed if desired. Please notice that these functionalities might not be working correctly anymore.

Contents:
---------

.. _GeneralUsage:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Installation and Usage

  GeneralUsage/*
  
.. _BeRoboy:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: BeRoboy
  
  BeRoboy/*  

.. _StateVisualization:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: State Visualization
  
  StateVisualization/*
  
.. _StateVisualization:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Moving Roboy
  
  MovingRoboy/*
  
.. _ModelWorldLoader:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Model Download & Update Pipeline
  
  ModelWorldLoader/*
      
.. _Pabi:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: PaBi VR
  
  Pabi/*
  
.. _Implementation
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Implementation
  
  Implementation/*

.. _Troubleshooting
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Troubleshooting
  
  Troubleshooting/*


.. _DevelopmentGraphs:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Development Graphs

  DevelopmentGraphs/*

.. toctree::
	:maxdepth: 1
	
	presentations
  
.. toctree::
   :maxdepth: 1

   about-arc42

   
   