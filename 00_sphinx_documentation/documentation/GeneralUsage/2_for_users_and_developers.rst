For Users and Developers
========================

For the user
------------

The user can simply start exploring the virtual environment without further ado. One of RoboyVR design goals is to have a user friendly and intuitive interface with which one can easily interact. Therefore, the user in the virtual environment does not need to be familiar with explicit commands or structures, he or she can simply explore the scene, experiment with all controllers and possible options without fear of issues or runtime failures. Yet, it does no harm to have a basic understanding of how the HTC Vive and its tracking mechanism work.

Putting on the head mounted display in a way that fits the user, he or she can adjust the distance from the lenses to the eyes as well as the distance between the lenses itself. These tweaks help immensely when it comes to maintaining a sharp field of view and having the full view frustrum. 

For the developer
-----------------

RoboyVR uses Unity3D to create an immersive and exciting virtual environment. Expierence with Unity is recommended. 
Unity natively relies on C#, so knowledge in this field is highly advised. Otherwise see `Unity3D <https://unity3d.com/>`_.

The Roboy simulations which run in Gazebo is written in C++, which normally load a world and model from an .sdf file, any additional plugins (written in C++) specified in these files and then simulate the given scene. For starting the simulation you should be familiar with Linux/Ubuntu.

The following links can be seen as a guideline, of course you can do
the research by yourself.

- Unity provides a lot of tutorials for the editor and the API with code samples and videos: https://unity3d.com/de/learn/tutorials
- The UnifyWiki has a lot of example scripts for all kind of extensions: http://wiki.unity3d.com/index.php/Main_Page
- StackOverflow is a forum where you can search for answers regarding your coding problems: http://stackoverflow.com/
- UnityAnswers, similar to StackOverflow but only for Unity specific questions. The community is not as active and most questions
  are really basic, so bear that in mind: http://answers.unity3d.com/
- As we use ROS and our own custom messages, it is important to understand how ROS works and how ROS messages are built: http://wiki.ros.org/
- Gazebo simulations rely on gazebo libraries, which are documented here: http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/index.html

If you have any further questions about the project, feel free to contact the Roboy team, which will connect you with the people in charge. 
