Implementation Documentation
============================
.. (comment ) Chapter

The implementation can be divided in the three main categories front-end, core and back-end. 

Front-End
---------
.. Section

 	
General UI Elements
~~~~~~~~~~~~~~~~~~~
.. Subsection

Mode Manager
************
.. last super class
.. elements following

.. doxygenclass:: MiddlewareManager
  :members:
  :private-members:

.. doxygenclass:: ControlManager
  :members:
  :private-members:
  
.. doxygenclass:: OverviewManager
  :members:
  :private-members:
  
Screen Related
**************

.. doxygenclass:: ScreenTabManager
  :members:
  :private-members:

.. doxygenclass:: ScrollViewFunctionalities
  :members:
  :private-members:
  
.. doxygenclass:: RotateWithView
  :members:
  :private-members:
  
Others
******

Also have a look at core components, as a definite distinction is not possible

.. doxygenclass:: RayOrder
  :members:
  :private-members:

.. doxygenclass:: DisplayManager
  :members:
  :private-members:
  
.. doxygenclass:: ExampleFunction
  :members:
  :private-members:
  
.. doxygenclass:: SmoothMovement
  :members:
  :private-members:
  
Core
----
.. Section

Also have a look at front-end re components, as a definite distinction is not possible
 	
Database
~~~~~~~~
.. Subsection


.. doxygenclass:: VRUILogic
  :members:
  :private-members:
  
Data Types Related
~~~~~~~~~~~~~~~~~~

.. doxygenclass:: DummyStates
  :members:
  :private-members:
  
.. doxygenclass:: Notification
  :members:
  :private-members:
  
.. doxygenclass:: NotificationListButton
  :members:
  :private-members:
  
.. doxygenclass:: AdditionalInfoScreenManager
  :members:
  :private-members:
  
.. doxygenclass:: Tendon
  :members:
  :private-members:
  
  
Back-End
--------

Database
~~~~~~~~
.. Subsection

.. doxygenclass:: ROSBridgeLib::custom_msgs::TendonInitializationMsg
  :members:
  
  :private-members:
.. doxygenclass:: ROSBridgeLib::custom_msgs::TendonUpdateMsg
  :members:
  
The structure of the following class is the same for all different notification types for now. 
(Here Error msg shown)
  
.. doxygenclass:: ROSBridgeLib::custom_msgs::WarningMsg
  :members:
  :private-members: