Public Interfaces
=================

ROSBridgeLib
^^^^^^^^^^^^

We use the following template from github for the ROSBridge: https://github.com/michaeljenkin/unityros.

Basically the ROSBridge consists of three different parts:

1) ROSBridgeWebSocketConnection
2) ROSBridgeMsg
3) ROSBridge Actor aka Subscriber, Publisher and Service

ROSBridgeWebSocketConnection
****************************

.. doxygenclass:: ROSBridgeLib::ROSBridgeWebSocketConnection
  :members:
  :private-members: