#roboyVR™


**How it works:**  
Roboy and its behavior is simulated on the virtual machine via ROS. Important information  
regarding roboy's movement are then sent through a ROSbridge(e.g. messages) towards Unity.  
In Unity roboy is rendered and constantly updated concerning positions, rotations, etc.  
With the help of a VR-Headset you can watch roboy move around in a virtual space.

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

1) Open Terminal and install the following packages

```
#!bash
sudo add-apt-repository -y ppa:letrend/libcmaes
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt install libcmaes
sudo apt-get install ros-kinetic-desktop-full
sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
sudo apt install ros-kinetic-ecl-geometry
sudo apt install libncurses-dev
sudo apt-get install catkin
sudo apt-get install git
```

2) Clone the git repository into a ros working space

```
#!bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Roboy/roboy-ros-control --recursive
```

3) Get additional dependencies

```
#!bash
cd roboy-ros-control
git submodule update --init --recursive
cd src/flexrayusbinterface
sudo dpkg -i lib/libftd2xx_1.1.12_amd64.deb
```

4) Source the setup.bash

```
source /opt/ros/kinetic/setup.bash
cd ~/ros_ws
catkin_make
```

5) OPTIONAL: add this to your bash script (otherwise you have to type this commands in every new terminal window)

```
echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc
```

6) Create symlinks for gazebo to your roboy models

```
#!bash
cd ~
mkdir -p ~/.gazebo/models
ln -s ~/ros_ws/src/roboy-ros-control/src/roboy_models/legs_with_upper_body ~/.gazebo/models/
```

7) Install rosbridge

```
#!bash
sudo apt install ros-kinetic-rosbridge-suite
```

### Part 1.3: Run rosbridge and roboySimulation

```
#!bash
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun roboy_simulation VRRoboy
```
### Part 2.1 Install Unity3D

1) Download Unity3D (last working version with this Project 5.4.3 : https://unity3d.com/de/get-unity/download/archive)

2) Install Unity3D. Make sure to choose also standalone build. Visual studio is recommonded to use with Unity3D, as it is free and more user friendly than MonoDevelop.

3) Open the project in Unity. Unity is organized in **Scenes**. (https://cloud.githubusercontent.com/assets/10234845/21025492/f72657fc-bd88-11e6-912e-877ba72d782e.png) To see the simulation from the VM in Unity open the **SimulationScene**.

### Part 2.2 SimulationScene ###

In the simulation scene you can see the simulation from the VM in Unity. To do that you need to pass the IP adress of the VM to the **RoboyManager**. (https://cloud.githubusercontent.com/assets/10234845/21025737/da6cda5e-bd89-11e6-8755-af5fbf4a748b.png)

You also need to drag the Roboy prefab into the RoboyManager if it is not already done. Each roboy model is tagged as *RoboyPart*. If you import new models for roboy you need to change the tag accordingly and change the roboy prefab.(https://cloud.githubusercontent.com/assets/10234845/21025736/da6bcb1e-bd89-11e6-820d-be7b42853697.png)

You can reset the simulation with **R** but you can also change the key in *RoboyManager*. To get a better view of the simulation we recommend to set the simulation to slow motion in rviz in the VM.

### Part 2.3 ViveScene ###

In the vive scene you can see roboy with the HTC Vive in VR. You can select each roboy part individually with the controller through a raycast. In the **Hierarchy** window you can see the *SelectorTool* attached to the right controller. This object has a line renderer component attached to it so you can see where you are pointing. Currently you can select a roboy part with the trigger on the back of the controller. (https://cloud.githubusercontent.com/assets/10234845/21025739/da714120-bd89-11e6-93ee-45949cf7dfc8.png)

Each roboy part has a *Selectable Object* script attached to it. You can choose two materials for visual feedback. (https://cloud.githubusercontent.com/assets/10234845/21025739/da714120-bd89-11e6-93ee-45949cf7dfc8.png)

### Update roboy models ###

In /roboyVR/Assets/RoboyModel/OriginModels there is a script **meshDownloadScript.py**. It downloads roboy models from https://github.com/Roboy/roboy_models/tree/master/legs_with_upper_body/cad. You need blender and python for this to work. You can use the template **runScript** bat file for Windows. 

The format:
```
start "" "pathToBlender/blender.exe" -P "pathToScript\meshDownloadScript.py" **meshes name seperated by ',' without whitespace and file format**
```

Example:

```
start "" "C:\Programs\Blender\blender.exe" -P "C:\Documents\roboyVR\Assets\RoboyModel\OriginModels\meshDownloadScript.py" hip,torso,thigh_left,head
```
