#roboyVR™

[![RoboyVR techdemo](https://raw.githubusercontent.com/sheveg/roboyVR/164b1f24ddfb1b060015f2370fcedeca5a160d78/Assets/thumbnail.png)](https://www.youtube.com/watch?v=4lgiljctMw8)

**How it works:**  
Roboy and its behavior is simulated on the virtual machine via ROS. Important information  
regarding roboy's movement are then sent through a ROSbridge(e.g. messages) towards Unity.  
In Unity roboy is rendered and constantly updated concerning positions, rotations, etc.  
With the help of a VR-Headset you can watch roboy move around in a virtual space.

Check out the whole large-scale documentation of roboyVR here:
https://readthedocs.org/projects/roboyvr-experience/

## Setup
This tutorial will help you setup roboyVR with all necessities it comes with.  

### Part 1.1: Setup Virtualbox with Ubuntu

1) Download and install Virtualbox for your OS https://www.virtualbox.org/

2) Download Ubuntu 16.04 (64bit) https://www.ubuntu.com/download/desktop

3) Mount the .iso and setup Virtualbox with the following settings (if available):

* 4 cores (Settings->System->Processor)
* 6 GB of RAM (Settings->System->Motherboard)
* 128 MB of VRAM (Settings->Display->Screen)
* 30 GB HDD space (Settings->Storage)

4) Set network settings to Bridged-Adapter or Host-Only Adapter  


### Part 1.2: Install ROS & Roboy on Ubuntu

Go to the main Roboy repository and follow the instructions of the [ReadMe](https://github.com/Roboy/Roboy/blob/master/README.md)

### Part 1.3: Run rosbridge and roboySimulation

```
#!bash
source /path-to-roboy-ros-control/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun roboy_simulation VRRoboy
```
### Part 2.1: Install Unity3D

1) Download Unity  
(latest working version with roboyVR is 5.4.3: https://unity3d.com/de/get-unity/download/archive)

2) Install Unity  
During the install process make sure to check also the standalone build option.  
Visual studio is recommended to use with Unity3D, as it is free and more user friendly than  
MonoDevelop (standard option).

3) Open the project in Unity  
Download or clone this github repository (master branch) to your system on open it in Unity via **Open Project** (git clone -b WS16/17 https://github.com/sheveg/roboyVR.git). Unity is organized in **Scenes**. In order to watch the simulation in Unity which is running on the VM (in gazebo), open the **SimulationScene**.  
![Scene overview](https://cloud.githubusercontent.com/assets/10234845/21025492/f72657fc-bd88-11e6-912e-877ba72d782e.png "Scene overview") 

### Part 2.2: SimulationScene ###

In the SimulationScene you can observe the simulation from the VM within Unity. To do that you need to communicate the IP adress of your VM towards **RoboyManager**. The IP information is quickly found in Ubuntu by clicking on the two arrows pointing in opposite directions, right next to the system time. Afterwards a drop down menu will open, click on connection information. Remember the IP and paste it in the respective field in Unity.  
![IP address](https://cloud.githubusercontent.com/assets/10234845/21025737/da6cda5e-bd89-11e6-8755-af5fbf4a748b.png "IP address")  
  
You also need to drag the roboy prefab onto the RoboyManager if it is not already done. Each roboy model is tagged as a *RoboyPart*. If you import new models for roboy you need to change the tag accordingly and change the roboy prefab.

![Tag assignment](https://cloud.githubusercontent.com/assets/10234845/21025736/da6bcb1e-bd89-11e6-820d-be7b42853697.png "Tag assignment")

You can reset the simulation with the **R** key, you can also change the key in *RoboyManager*. To get a better view of the simulation we recommend to set the simulation to slow motion in rviz in the VM:
* If you want to start rviz, open a terminal (in the VM) and simply type **rviz**
* Set Fixed Frame to World (Displays->Fixed Frame)
* Add a marker (Add(Button)->marker)
* Add walking plugin (Panels->Add New Panel->WalkingPlugin)
* Turn slow motion on (within the walking plugin, it is a toggle button) 

### Part 2.3: ViveScene ###

In the ViveScene you can view roboy with the HTC Vive™ in virtual reality. You can select each roboy part individually with the controller through a raycast. In the **Hierarchy** window you can see the *SelectorTool* attached to the right controller.  
![Selector Tool](https://cloud.githubusercontent.com/assets/10234845/21025739/da714120-bd89-11e6-93ee-45949cf7dfc8.png "Selector Tool")This object has a line renderer component attached to it so know where you are pointing the device at. Currently you can select a roboy part with the trigger on the back of the controller. Each roboy part has a *Selectable Object* script attached to it. You can choose from two materials for visual feedback.  
![Materials selection](https://cloud.githubusercontent.com/assets/10234845/21025738/da6c4c06-bd89-11e6-883c-db8d20462f42.png "Materials selection")

### Update roboy models ###

In /roboyVR/Assets/RoboyModel/OriginModels there is a script **meshDownloadScript.py**. When executed, it downloads roboy models from this location:  
https://github.com/Roboy/roboy_models/tree/master/legs_with_upper_body/cad.  

After the download process is complete the models will be converted by blender so that they work fine with unity (.fbx format). Obviously you need blender and python installed on your system so that the script can do it's work. You can use the template **runScript** bat file for Windows. 

The format:
```
start "" "pathToBlender/blender.exe" -P "pathToScript\meshDownloadScript.py" **meshes name seperated by ',' without whitespace and file format**
```

Example:

```
start "" "C:\Programs\Blender\blender.exe" -P "C:\Documents\roboyVR\Assets\RoboyModel\OriginModels\meshDownloadScript.py" hip,torso,thigh_left,head
```
