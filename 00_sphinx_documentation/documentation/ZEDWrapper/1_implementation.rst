Implementation
==============

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

To retrieve the mesh you just have to call the functions to get the mesh arrays and save them in the mesh representation in Unity.
This means you have to import the needed functions and call them. In our example we call the mapping loop in a seperate thread for
performance reasons.