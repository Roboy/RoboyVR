ROSBridge
=========

ROS overview
^^^^^^^^^^^^

The ROSBridge is a main part of this project as it connects the Unity project on Windows and the simulation
on Ubuntu. We recommend to do the tutorials `here <http://wiki.ros.org/ROS/Tutorials>`_ to get a basic understanding
of ROS. The short version follows below.

The ROS core establishes connections between clients and creates topics.
Topics are basically like an adress where you can decide send or receive data from.
For that you have three main types of communication in ROS.

Publisher
*********

The publisher *sends* data to a topic. When you create the topic you have to tell ROS of which type the data
will be. Therefore at first you have to *advertise* which creates a topic. Afterwards you can *publish*
data with the defined message type.

Subscriber
**********

A subscriber *receives* data from a topic. So every time data is published to the subscribed topic you get
a message. Therefore you have to define a function which will be called when a message is received. This
function has to have the message type as argument parameter so you can parse the message and use it
to do stuff.

Service
*******

A service is a little bit different compared to the publisher and subscriber pattern. It works basically like a function call
when the service is called via a topic. This means that you as the *service client* call the *service server*
with the needed data depending on the certain service via a *service request*. Then the server responds to you via a
*service response*. This is useful when you need to do certain tasks only on rare occasions e.g. reset a world.

Now that you know how to communicate you need to create messages which fit your task.
As you want to communicate between Ubuntu and Windows via a ROSBridge you need to create the messages on both sides.
On the Ubuntu side it is pretty straightforward, just have a look at the tutorial examples. The conversion
from the message type to the `YAML format <https://en.wikipedia.org/wiki/YAML>`_ is done automatically for you.

Implementation
^^^^^^^^^^^^^^

However, on the Windows side it is more complicated. You have to do the conversion by yourself.
This is why if you define your own custom message you need to define a ToYAMLString() method in the message class
and implement it *if you want to publish data of this message type*. This means for messages which are only received
it is sufficient to implement only the constructor method with type *JSONNode* as parameter.

.. code-block:: c#

  public CustomMessage(JSONNode msg)
  {
      // parse message here
  }

And for the conversion:

.. code-block:: c#

  public override string ToYAMLString()
  {
	  // format the message ... 
      return string_in_yaml_format;
  }
  
As summary this means, that you need to:

1) Create the message on Ubuntu side
2) Create a class for the message on Windows side
3) Derive the message class from *ROSBridgeMsg* so the ROSBridge knows that this is a message
4) Implement the constructor and/or the ToYAMLString method depending on your needs

Now that you have the message you need to create a subscriber, publisher or or a service. To do this on the Ubuntu side
just follow the tutorial. In Unity you need to create a class for each one of these.
Each of this ros types have different requirements to be valid but each of these have to derive from the
corresponding base class, e.g. CustomPublisher : ROSBridgePublisher

The publisher needs to have:

1) GetMessageType()
2) GetMessageTopic()

The subscriber needs to have:

1) CallBack()
2) GetMessageType()
3) GetMessageTopic()
4) ParseMessage()

The service needs to have:

1) ServiceCallBack

All methods must be public and static. If you forget to implement one of these the ROSBridge will throw an expcetion
which tells you which method is missing.


Wrapper
^^^^^^^

Originally we got the template from `here <https://github.com/michaeljenkin/unityros>`_. To make it more
usable and developer-friendly we wrote a wrapper. The Wrapper consists of two main classes.

*ROSObject*

A ROSObject retrieves all ros types on the gameObject e.g. publisher at the start of the scene.
On enable it tells the *ROSBridge* to add himself to the ROSBridge and to remove himself on disable.

*ROSBridge*

The ROSBridge is the main part of the wrapper. It forwards the needed calls to *ROSBridgeWebsocketConnection* which establishes a websocket and is the actual backend.

Scene Setup
^^^^^^^^^^^

When you created your custom publishers etc. and messages you need to edit your scene such as the ROSBridge recognizes that you want to send/receive data.
First you need to either drag the ROSBridge prefab or create it yourself and enter the IP adress and port of the ros core you want to connect to.

.. figure:: ../images/rosbridge.png
	:align: center
	:alt: ROSBridge
	
	ROSBridge

Then attach to the gameObject which should receive/send data a ROSObject script and all needed ros types.

.. figure:: ../images/ros_object.png
	:align: center
	:alt: ROSObject
	
	ROSObject

Congrutalations, you finished the setup!
