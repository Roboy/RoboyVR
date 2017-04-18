.. Software Documentation template master file, created by
   sphinx-quickstart on Fri Jul 29 19:44:29 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome roboyVR documentation!
===========================================================

The goal of this project is to provide an interactive VR interface for roboy simulations of all kind.
This means that you can include your own roboy simulation, send your data to the interface and look
at the result in a VR environment. On top of that you can send additional data about roboy and visualize 
the data in VR on a Graphical User Interface.

Currently the project can render roboy with his pose and generate random data about his motors to visualize them.
Our next tasks are as follow:

- Use real motor data and visualize that.
- Implement an interface to track the newest models and automate the process of creating the model in Unity.
- Implement an interface to record a simulation with all the data and save/ load it on runtime.
- Make the project completely Plug&Play meaning that you can send all kinds of data with a given format.


.. _background_prerequisits:

Relevant Background Information and Pre-Requisits
--------------------------------------------------

As our project is completely build in the Unity3D engine you should at least know the basics of the editor
and the Unity API. We use C# in all of our code, so the same applies here. The simulation itself is not
written by us, but you should have basics skills in Linux, C++ and ROS to make small changes in the simulation
for testing purposes and compile them. If you write your own simulation, you shoud know how to use the ROS API
with Gazebo and format your messages in a manner so you can parse them accordingly. You do not need any special
theoritical knowledge, it is pretty much straightforward. But you should be used to Object Oriented Programming.
  
The following links can be seen as a guideline, of course you can do
the research by yourself.

- Unity provides a lot of tutorials for the editor and the API with code samples and videos: https://unity3d.com/de/learn/tutorials
- The UnifyWiki has a lot of example scripts for all kind of extensions: http://wiki.unity3d.com/index.php/Main_Page
- StackOverflow best site ever: http://stackoverflow.com/
- UnityAnswers, kinda like StackOverflow but only Unity specific questions. The community is not as active and most questions
  are really basic, so bear that in mind: http://answers.unity3d.com/
.. _requirements_overview:

Contents:
---------

.. _usage:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Usage and Installation

  Usage/*

.. _ScopeContext:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Interfaces and Scope

  ScopeContext/*

.. _development:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Development

  development/*

.. toctree::
   :maxdepth: 1

   about-arc42
