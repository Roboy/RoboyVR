Introduction
============  

.. figure:: images/thinner.*
   :align: center
   :alt: Image could not be loaded
   
Usage
-------

The Virtual Reality user interface will display all given and desired data in a structured environment to help both developers and external visitors to gain further insight into Roboy and how he works. For the developer side, it is important to display the data in a coherent way to clearly communicate the current state of Roboy to aid in implementation and debugging scenarios. Goals for the visitor include designing a visually appealing interface which does not overload the user with unnecessary and misleading information, but provides selected information and explanations to satisfy the visitor's interest. Important aspects include structuring and grouping the given data in sets, between which the user can change and which he can activate dynamically, providing an intuitive control system which does not need further explanation and visualizing the given data in a clear and understandable way. 

Structure
---------

In general, the Scene contains two types of objects:

- Scene related Objects: Roboy, the Background, the Camera
- UI related Objects: Canvases, screens, container objects, UI elements such as panels, buttons, images

The UI has three main layers:

- Front-end: Containing all UI objects
- Core: Containing logic, updating methods and operating on given data set
- Back-end: Provider of data through a connection to ROS (or method dummies sending fake data)

Current Implementation
----------------------
See the user manual for further information. 