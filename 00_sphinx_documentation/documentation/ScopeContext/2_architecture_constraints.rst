Architecture Constraints
========================


.. csv-table:: Hardware Constraints
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "HTC Vive", "We need user position tracking and movement tracking."
  
  "ZED", "For spatial mapping and a live stream of the environment."

.. csv-table:: Software Constraints
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "Unity3D", "Unity provides an interface for the HTC Vive with the steamVR plugin. On top of that it renders the simulation."
  "Gazebo&ROS", "The simulation uses both systems."
  "OracleVM", "We use the VM for running Ubuntu on the same machine. You can also just use Ubuntu on a separate machine."
  "Blender", "We used blender to convert the roboy models so that Unity can import them."

.. csv-table:: Additional Plugins
  :header: "Constraint Name", "Description"
  :widths: 20, 60
  :align: right

  "ROSBridge", "It connects the simulation on Ubuntu with Unity on Windows."
  "steamVR", "We use this interface to use the API of the HTC Vive."
  "ZED", "This interface connects the ZED (Roboy's Eyes) with Unity."
  "PyXB", "This is used for reading XML files in the Model/World Updater."

.. csv-table:: Operating System Constraints
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "Windows 10", "We did not test it yet on other Windows versions. It may also work on older machines."
  "Ubuntu 16.04", "The simulation runs on Ubuntu."

.. csv-table:: Programming Constraints
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "C++", "The simulation is written in C++."
  "C#", "Unity uses C# as the standard programming language."
  "Python", "We use Python with the Blender API to automate the process of converting the roboy models."
