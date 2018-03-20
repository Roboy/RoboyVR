Model Spawner
=============

It is possible to insert and remove numerous models from the virtual world
simply by point and click. For this purpose we designed and implemented a seperate tool which you can
selected from the GUI MODE hand controller (selectionwheel is activated by the side button).

.. figure:: images/controller_layout.*
  :alt: Controller layout
  
  Overview of the button mapping for the Vive controllers.

Just choose the model you want to insert and click on a free space on the ground. This model will be inserted
in the world of Unity, but in the simulation PaBi will not be inserted as well as the surprise model.
This is due to missing content in the gazebo model structure on the simulation side.

In addition the models get inserted but their poses are not updated by the simulation because of lacking infrastructure. Removing models is partly possible, even though it does not work with all of the models, e.g. the surprise model and PaBi will not work.
