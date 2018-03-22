ZED Wrapper
============

**NOTICE : LEGACY CODE. NOT MAINTAINED**

**Introduction**
The ZED camera works together with the API to create a spatial representation of the environment. It can also be used to track the position of the user but for that we already use the HTC Vice. Unfortunately the the Unity plugin for the ZED API does not expose the spatial mapping API. But we wanted to do a case study to test the possibilities and performance of the camera. Therefore we wrote our own wrapper which maps the c++ API to Unity. You can find the c++ code `here <https://github.com/sheveg/UnityZEDWrapperCPP>`_
and the c# code `here <https://github.com/sheveg/UnityZEDWrapperCSharp>`_.

**Implementation**
The difficult part was to figure out how to successfully transfer the results from the c++ dll to Unity.
After a lot of trial and error we settled with the approach to send data to Unity in sperate arrays.

The spatial scan API saves the result in an internal `*Mesh* <https://www.stereolabs.com/developers/documentation/API/classsl_1_1Mesh.html>`_ class. This class has various arrays for the vertices, triangles, normals and uvs.
On top of that it structures the vertices in `*Chunks <https://www.stereolabs.com/developers/documentation/API/classsl_1_1Chunk.html>`_ to make things faster. This means that every time the camera sees a new part
it saves it in a chunk instead of going through the whole vertices array. So to send the retrieved data the plugin offers
functions to retrieve the mesh arrays. So the wanted usage of the plugin is as follows:

1) Start the spatial mapping process.
2) Run the main spatial mapping loop each frame/time step.
3) Retrieve the mesh each time you get a new update to see the live progress.
4) Stop the spatial mapping process.
5) Get the final mesh and the texture for color information.

To retrieve the mesh you just have to call the functions to get the mesh arrays and save them in the mesh representation in Unity. This means you have to import the needed functions and call them. In our example we call the mapping loop in a seperate thread for performance reasons.

**Results**
There is a good and a bad message. The good message first. It works, kinda. So the result of the scan itself is somewhat right. However, there is a problem with the scaling and orientation. It is way too big, of factor 1000, and it is turned upside-down. We tried different approaches but did not find out the reasons for these problems. 

The bad message is that the live progress has a lot of artifacts. This may be because of the multithreading approach.
We tried to make it threadsafe via locking and only reading when the mappingLoop is not writing but for some reason it did not solve the artifacts completely but rather just reduced them. This problem makes the usage of this appraoch in VR impossible. The user would fill sick and the vision of him would be completely clustered by seemingly random triangles.

Another problem is that, as the standard plugin and the extension hold both a reference to the zed camera they cannot be used together. Therefore you cannot start the spatial mapping process while getting a live stream of the camera. This is unfortunate as the wanted result was exactly this behaviour.

Summerized, in the current state it is more of a presentation that it can work in Unity with a decent performance but lacks stability and robustness to use it in a real-case scenerio.