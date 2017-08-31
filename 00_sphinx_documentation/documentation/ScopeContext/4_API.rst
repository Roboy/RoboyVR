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

ROSBridgeMsg
************

.. doxygenclass:: ROSBridgeLib::ROSBridgeMsg
  :members:
  :private-members:
  
As every type of ROSBridgeMsg should derive from this class, here is an example how an actual implementation looks like.

.. doxygenclass:: ROSBridgeLib::turtlesim::PoseMsg
  :members:
  :private-members:

Managers
^^^^^^^^

RoboyManager
************

.. doxygenclass:: RoboyManager
  :members:
  :private-members:
  
InputManager
************

.. doxygenclass:: InputManager
  :members:
  :private-members:

ModeManager
***********

.. doxygenclass:: ModeManager
  :members:
  :private-members:
  
SelectorManager
***************

.. doxygenclass:: SelectorManager
  :members:
  :private-members:
  
BeRoboyManager
**************

.. doxygenclass:: BeRoboyManager
  :members:
  :private-members:
  
ViewSelectionManager
********************

.. doxygenclass:: ViewSelectionManager
  :members:
  :private-members:

Tools
^^^^^

ControllerTool
**************

.. doxygenclass:: ControllerTool
  :members:
  :private-members:
  
SelectorTool
************

.. doxygenclass:: SelectorTool
  :members:
  :private-members:
  
ShootingTool
************

.. doxygenclass:: ShootingTool
  :members:
  :private-members:

GUIController
*************

.. doxygenclass:: GUIController
  :members:
  :private-members: