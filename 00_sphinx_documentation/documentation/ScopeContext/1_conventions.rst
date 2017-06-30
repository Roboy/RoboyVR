Conventions
-----------
We follow the coding guidelines:

.. csv-table:: Coding Guidelines
   :header: "Language", "Guideline", "Tools"
   :widths: 20, 30, 30

   "Python", "https://www.python.org/dev/peps/pep-0008/", ""
   "C++", "http://wiki.ros.org/CppStyleGuide", "clang-format: https://github.com/davetcoleman/roscpp_code_format"
   

The project follows custom guidelines.

Regions
=======

In general all scripts are ordered in regions:

  - PUBLIC_MEMBER_VARIABLES
  - PRIVATE_MEMBER_VARIABLES
  - UNTIY_MONOBEHAVIOUR_METHODS
  - PUBLIC_METHODS
  - PRIVATE_METHODS
  
**However**, in some cases it does make sense do deviate from this structure.

1. Delete empty regions!

It is okay to have empty regions if you plan on filling this region in the near future (READ: in the next seven days).
But to avoid whitespace code remove empty regions.

2. Add extra regions!

If you have a lot of variables of functions which are bundled together through some kind of logic then it may be beneficial for the readability
to structure them in a seperate region, e.g. variables which are private but are marked with **[SerializeField]** so you can adjust them in the editor and are only used
once then you can create a region **EDITOR_VARIABLES**.

3. Delete regions in small classes!

If you have a class with only two or three small functions/ variables then just write the code without the regions at all.

4. Use your brain!

As we are not the masterminds of the universe there are a ton of other cases where it does make more sense to use your **own* region logic. Feel free to extend/ adjust
this list if you have a better approach.

**Region Content**

What goes where should be obvious. In the UNTIY_MONOBEHAVIOUR_METHODS region write your code in the same order as the unity functions are called, meanining:

..  code-block:: c#
  
  void Awake(){}
  
  void Start() {}
  
  void OnEnable() {}
  
  void OnDisable() {}
  
  void FixedUpdate() {}
  
  void Update() {}
  
  void LateUpdate() {}
  
These are of course not all Unity functions but the most common ones. Note that **OnEnable()** is actucally called before **Start()** but more often than not
the logic in **OnEnable()** is coupled with **OnDisable()**. If this is not the case for you then change the order as you need it. For further reading of the execution order
click `here <https://docs.unity3d.com/Manual/ExecutionOrder.html>`_ .

OLD_DOCU REWRITE THIS SHIT
========================

1. All scripts are structured like this:
  a. The script is ordered in regions: 
    - PUBLIC_MEMBER_VARIABLES
    - PRIVATE_MEMBER_VARIABLES
    - UNTIY_MONOBEHAVIOUR_METHODS
    - PUBLIC_METHODS
    - PRIVATE_METHODS
  b. In PUBLIC_MEMBER_VARIABLES you have define at first your properties and then public variables.
  c. In PRIVATE_MEMBER_VARIABLES you have define at first your serialized private variables and then the normal ones.
  d. In UNTIY_MONOBEHAVIOUR_METHODS the order is as follows: Awake, Start, OnEnable, OnDisable, Update
2. All variables and functions where it is not instantly clear what it does, have to be commented with a summary.
3. Make variables only public if they need to be. Mark variables as Serializable when you need to edit them in the editor.
4. The capitalization follows a specific set of rules:
  - public variables and properties start with an uppercase
  - private variables and properties start with a lowercase
  - public functions start with an uppercase
  - private functions start with an lowercase
5. Coroutines which are accessed in other classes must have a public interface.
6. When you store components in a variable, which are directly on the object itself, put a [RequireComponent(typeof(ComponentType))] on top of the class.

We include a template class with all rules implemented.

.. doxygenclass:: TemplateClass
  :members:
  :private-members:

