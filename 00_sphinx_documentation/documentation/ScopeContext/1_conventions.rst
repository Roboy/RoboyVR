Conventions
-----------
We follow the coding guidelines:

.. csv-table:: Coding Guidelines
   :header: "Language", "Guideline", "Tools"
   :widths: 20, 30, 30

   "Python", "https://www.python.org/dev/peps/pep-0008/", ""
   "C++", "http://wiki.ros.org/CppStyleGuide", "clang-format: https://github.com/davetcoleman/roscpp_code_format"
   

The project follows custom guidelines:

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

