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

**1. Delete empty regions!**

It is okay to have empty regions if you plan on filling this region in the near future (READ: in the next seven days).
But to avoid whitespace code remove empty regions.

**2. Add extra regions!**

If you have a lot of variables of functions which are bundled together through some kind of logic then it may be beneficial for the readability
to structure them in a seperate region, e.g. variables which are private but are marked with `[SerializeField] <https://docs.unity3d.com/ScriptReference/SerializeField.html>`_ so you can adjust them in the editor and are only used
once then you can create a region **EDITOR_VARIABLES**.

**3. Delete regions in small classes!**

If you have a class with only two or three small functions/ variables then just write the code without the regions at all.

**4. Use your brain!**

As we are not the masterminds of the universe there are a ton of other cases where it does make more sense to use your **own** region logic. Feel free to extend/ adjust
this list if you have a better approach.

**Region Content**

What goes where should be obvious. In the UNTIY_MONOBEHAVIOUR_METHODS region write your code in the same order as the unity functions are called, meaning:

..  code-block:: c#
  
  void Awake(){}
  
  void Start() {}
  
  void OnEnable() {}
  
  void OnDisable() {}
  
  void FixedUpdate() {}
  
  void Update() {}
  
  void LateUpdate() {}
  
These are of course not all Unity functions but the most common ones. Note that **OnEnable()** is actually called before **Start()** but more often than not
the logic in **OnEnable()** is coupled with **OnDisable()**. If this is not the case for you then change the order as you need it. For further reading of the execution order
click `here <https://docs.unity3d.com/Manual/ExecutionOrder.html>`_.

Also put **static** variables/ methods on top of the respective region.

..  code-block:: c#
  
  #region PUBLIC_MEMBER_VARIABLES
  
  public float static MagicNumber = 42f;
  
  public float Speed;
  
  #endregion // PUBLIC_MEMBER_VARIABLES
  
  #region PRIVATE_MEMBER_VARIABLES
  
  private float const m_PI = 3.14;
  
  private int m_ElementCount;
  
  #endregion // PRIVATE_MEMBER_VARIABLES

Naming
======

All code of this project which was implemented by ourselves follows also some specific patterns. In general, names should not contain underscores or numbers, exceptions see below.
Addionally there is one important rule:

**Write stuff out!**

As we live in the glory days of code completion from the IDE you actually can give long names to your classes, variables etc.
Therefore avoid using shortened names or abbrivations. (Exception: Widely-known ones like Xml, Html, Fbx etc.)

.. code-block:: c#
  
  // Correct
  public float EnemyHealth;
  public float DamageOverTime;
  
  // Avoid
  public float elemCount;
  public float dmgInAMinByTypeAInLvlKek

Classes
^^^^^^^

Classes should use `PascalCasing <https://en.wikipedia.org/wiki/PascalCase>`_.

..  code-block:: c#

  // Correct
  public class PlayerController
  {
    //...
  }
  // Correct
  private class PlayerHelper
  {
    //...
  }
  
  // Avoid
  public class Enemy_Controller
  {
    //...
  } 
  public class bullet01
  {
    //...
  }

On top of that scripts which implement the `Singleton <https://en.wikipedia.org/wiki/Singleton_pattern>`_ approach, the name should suffix "Manager".
Note that we have already a singleton class. At the same time, classes which are **not** a singleton should avoid using "Manager" in the name.
Most of the time what you want is then a **controller**.

.. code-block:: c#

  // Correct
  public class EnemyManager : Singleton<EnemyManager>
  {
    //...
  }
  // Correct
  public class UIController : Monobehaviour
  {
    //...
  }
  
  // Avoid: a singleton without "Manager" suffix!
  public class Robot : Singleton<Robot>
  {
	//...
  }
  // Avoid: not a singleton!
  public class LevelManager : Monobehaviour
  {
	//...
  }

Variables
^^^^^^^^^

**Public variables**

Public variables should also use **PascalCasing**.

..  code-block:: c#

  // Correct
  public float Width;
  public float Height;
  
  // Avoid
  public float scale;
  public float _size;
  
**Private and protected variables**

Private and protected variables should also use **PascalCasing** with a prefix "m\_" for *member variables*.

.. code-block:: c#
  
  // Correct
  protected string m_Name = "Simon";
  private string m_NickName = "The coding god";
  
  // Avoid
  private string heh = "heh?";
  
The reasoning behind this is so that you can differentiate between local and private variables at first glance.

**Local variables**

Local variables inside a method should use `camelCasing <https://en.wikipedia.org/wiki/Camel_case>`_.

.. code-block:: c#

  // Correct
  public void Heh()
  {
    string heh = "heh??????";
	//...
  }
  
  // Avoid
  public void BadHeh()
  {
    string m_Wut = "wut?";
    //...
  }

Methods
^^^^^^^

**Public methods**

Public methods should use PascalCasing.

.. code-block:: c#

  // Correct
  public void DoStuff()
  {
    //...
  }
  
  // Avoid
  public void doBadStuff()
  {
    //...
  }
  
**Private and protected methods**

Private and protected methods should use camelCasing.

.. code-block:: c#

  // Correct
  private void doPrivateStuff()
  {
    //...
  }
  
  // Avoid
  private void DoPrivateBadStuff()
  {
    //...
  }
  
**Coroutines**

`Coroutines <https://docs.unity3d.com/Manual/Coroutines.html>`_ are special methods in Unity. They cannot be started by a simple method call.
You must start them via **StartCoroutine()**. Therefore to make sure that future developers see at first glance whether your method is
a coroutine or not, name them with a "coroutine" suffix.

.. code-block:: c#

  // Correct
  private IEnumerator someCoroutine()
  {
    //...
  }
  
  // Avoid
  private IEnumerator badCode()
  {
    //...
  }

General rules
=============

If you follow the rules above you should be now a naming god. But as syntax is only one half of the equation here comes the second one: **semantics.**

1. Public variables/ methods should only be public if other scripts **really** need or may need access to this functionality.

It makes it easier to understand what your class actually does from the standpoint of other classes.

Variables which you need to be editable in the editor but are not actually accessed by other classes should be non public and marked with 
a `[SerializeField] <https://docs.unity3d.com/ScriptReference/SerializeField.html>`_ attribute. Note that not all types can be marked as serializable, 
e.g. properties of any kind are not. However, we have just the right custom attribute for this case: [ExposeProperty]. For this to work
mark the variable which is wrapped in the property as serializable but also hidden from the inspector by 
`[HideInInspector] <https://docs.unity3d.com/ScriptReference/HideInInspector.html>`_ so you do not see both, the property and the
variable in the editor.

.. code-block:: c#

  // Read only property exposed in the editor so we always see the current status for debug reasons
  [ExposeProperty]
  public int Status { get { return m_Status;} }
  
  [HideInInspector]
  private int m_Status = 0; 

2. Hide public variables when you do not want to change them in the editor.

This removes clutter from the inspector and makes it somewhat "safer". You can achieve this behaviour with the attribute [HideInInspector].
But be aware that if you first change the value of a variable in the inspector to something else than the default value given by code and
**then** add the attribute it **still** holds the new value although it is not visible in the inspector.

.. code-block:: c#

  // This will leave the public field serializable so it saves the state from the editor to playing but hide it in the inspector.
  [HideInInspector]
  public int RandomNumber = 42;
  
3. Use properties if changes in your variables trigger other logical changes.

Properties are a nice way to wrap a variable and add more functionality to this variable. For example if you have a player who has different states depending
on his current health, then you can write the logic or trigger the logic inside the property.

.. code-block:: c#

  public int Health 
  {
    get
	{
	  return m_Health;
	}
	set
	{
	  m_Health = Mathf.Max(0f, value);
	  // different callbacks depending on the health
	  if(m_Health < 80)
	    decreaseSpeed();
	  if(m_Health < 40)
	    setCanJump(false);
	  if(m_Health < 0)
	    dieMiserably();
	}
  }
  
  private int m_Health = 100;
  
4. Use the `[RequireComponent(typeof(ComponentType))] <https://docs.unity3d.com/ScriptReference/RequireComponent.html>`_ to signal a strict dependency on another component on the same gameObject.

If your script depends on another script to work properly on the same gameObject add this attribute on top of the class declaration.
This way every time your script is attached to another object the dependency is added automatically. It is useful e.g. if you have
a script which adds a listener to a button in **Start()**.

.. code-block:: c#
  
  [RequireComponent(typeof(Button))]
  public class CustomButton
  {
    void Start()
	{
	  GetComponent<Button>().onClick.AddListener(pickleRick);
	}
	
	void pickleRick()
	{
	  Debug.Log("I am a pickle Morty!");
	}
  }

5. Coroutines should have an intermediate function if accessed from other classes.

To avoid the situation that another developer tries to call your awesome Coroutine method without a **StartCoroutine** don't expose
coroutines but rather write an intermediate method which calls the desired coroutine.

.. code-block:: c#

  // Class MrMeeseeks
  public void AwesomeStuff()
  {
	StartCoroutine(awesomeStuffCoroutine());
  }
  
  private IEnumerator awesomeStuffCoroutine()
  {
    // magic
  }
  
  // Class Jesus
  void Start()
  {
    GameObject maria = new GameObject("Maria");
	MrMeeseeks meeseeks = maria.AddComponent<MrMeeseeks>();
	meeseeks.AwesomeStuff();
  }

6. With this knowledge take over the world.

Basic stuff actually. Ask simon how to do it.