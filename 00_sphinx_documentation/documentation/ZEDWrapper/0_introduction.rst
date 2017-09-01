Introduction
============

The ZED camera works together with the API to create a spatial representation of the environment. It can also be used to track the
position of the user but for that we already use the HTC Vice. Unfortunately the the Unity plugin for the ZED API does not expose
the spatial mapping API. But we wanted to do a case study to test the possibilities and performance of the camera. Therefore
we wrote our own wrapper which maps the c++ API to Unity. You can find the c++ code `here <https://github.com/sheveg/UnityZEDWrapperCPP>`_
and the c# code `here <https://github.com/sheveg/UnityZEDWrapperCSharp>`_.