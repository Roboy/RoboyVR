Context
--------

The core of RoboyVR renders and updates roboy's pose as its receiving data from the simulation
via ROS-messages. Additional information inside messages like current powerconsumption or motorforce
is displayed on an interactive GUI. Apart from that the user can actively manipulate the simulation
through various tools. On top of that the system can check for the latest roboymodel with the help of
github and update it if necessary.

.. figure:: images/uml_context.*
  :alt: Context overview.

  RoboyVR Experience has two neighboring systems. 
  Roboy simulation to receive pose data and Github for model updates.
  
  

