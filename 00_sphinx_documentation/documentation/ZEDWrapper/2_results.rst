Results
=======

There is a good and a bad message. The good message first. It works, kinda. So the result of the scan itself is somewhat right.
However, there is a problem with the scaling and orientation. It is way too big, of factor 1000, and it is turned upside-down.
We tried different approaches but did not find out the reasons for these problems. 

The bad message is that the live progress has a lot of artifacts. This may be because of the multithreading approach.
We tried to make it threadsafe via locking and only reading when the mappingLoop is not writing but for some reason it did not
solve the artifacts completely but rather just reduced them. This problem makes the usage of this appraoch in VR impossible.
The user would fill sick and the vision of him would be completely clustered by seemingly random triangles.

Another problem is that, as the standard plugin and the extension hold both a reference to the zed camera they cannot be used together.
Therefore you cannot start the spatial mapping process while getting a live stream of the camera. This is unfortunate as the wanted
result was exactly this behaviour.

Summerized, in the current state it is more of a presentation that it can work in Unity with a decent performance but lacks stability
and robustness to use it in a real-case scenerio.