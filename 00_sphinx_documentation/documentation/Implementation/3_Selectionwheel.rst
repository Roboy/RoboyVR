Selection Wheel
===============

The selection wheel class is a generic approach to bundle items into a circle based selection. You can of course modify the class
or create your own custom selection wheel but for the most part it should work out of the box if you define certain classes.
There are basically two main classes for the SelectionWheel items. At first, you need the actual item with working functionality.
In our case it is always a tool. This class needs an icon which will be the representation of the item in the selection wheel.

Then you need the UI counterpart item. As the SelectionWheel itself bundles the UI parts and calls the corresponding classes depending on the
action of the user. To be more specific, the UI part knows three states: *Unhighlighted, Highlighted and Selected*. Depending on the state change
it calls the corresponding method: Highlight(), Unhighlight() etc. In this class you have to define the behaviour of the UI, meaning what color
it should have depending on the state, possible animations, debugs etc. On top of that it should call the same method in the actual part which
triggers some logic change etc. For example in our case we have defined a SelectionWheel to change between Tools. This means that in the UI part
we just do basic color switches and in the actual part we call a method from *InputManager* to change the current tool to the selected one.