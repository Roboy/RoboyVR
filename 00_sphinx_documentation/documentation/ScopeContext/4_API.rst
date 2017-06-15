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
  
ROSBridgeActors
***************

.. doxygenclass:: ROSBridgeLib::ROSBridgePublisher
  :members:
  :private-members:
  
.. doxygenclass:: ROSBridgeLib::ROSBridgeSubscriber
  :members:
  :private-members:
  
.. doxygenclass:: ROSBridgeLib::ROSBridgeService
  :members:
  :private-members:
  
ROSBridgeLibExtension
^^^^^^^^^^^^^^^^^^^^^

We extended the library in the form that we implemented a singleton class to handle all ROSActors in the scene.

.. doxygenclass:: ROSBridge
  :members:
  :private-members:
  
.. doxygenclass:: ROSObject
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

Additional classes
^^^^^^^^^^^^^^^^^^

SelectableObject
****************

.. doxygenclass:: SelectableObject
  :members:
  :private-members:

SelectionPanel
**************

.. doxygenclass:: SelectionPanel
  :members:
  :private-members:  

Projectile
**********

.. doxygenclass:: Projectile
  :members:
  :private-members:


