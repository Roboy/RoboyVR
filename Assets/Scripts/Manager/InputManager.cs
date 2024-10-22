﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib; //TODO:only for test purposes, remove later
using System.Linq;

/// <summary>
/// InputManager holds a reference of every tool. On top of that it listens to button events from these tools and forwards touchpad input to the respective classes.
/// </summary>
public class InputManager : Singleton<InputManager>
{
    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// Possible touchpad positions.
    /// </summary>
    public enum TouchpadStatus
    {
        Right,
        Left,
        Top,
        Bottom,
        None
    }

    /// <summary>
    /// Public GUIController reference.
    /// </summary>
    public GUIController GUI_Controller
    {
        get { return m_GUIController; }
    }

    /// <summary>
    /// Public ViewController reference.
    /// </summary>
    public ViewController View_Controller
    {
        get { return m_ViewController; }
    }

    /// <summary>
    /// Public ModelSpawnController reference.
    /// </summary>
    public ModelSpawnController ModelSpawn_Controller
    {
        get { return m_ModelSpawnController; }
    }

    /// <summary>
    /// Public SelectorTool reference.
    /// </summary>
    public SelectorTool Selector_Tool
    {
        get { return m_SelectorTool; }
    }

    /// <summary>
    /// Public ShootingTool reference.
    /// </summary>
    public ShootingTool ShootingTool
    {
        get { return m_ShootingTool; }
    }

    /// <summary>
    /// Public TimeTool reference.
    /// </summary>
    public TimeTool TimeTool
    {
        get { return m_TimeTool; }
    }

    public HandTool HandTool
    {
        get { return m_HandTool; }
    }

    [Header("Controller Description")]


    /// <summary>
    /// Reference to prefab which writes "mode" on one side of the controller
    /// </summary>
    public GameObject m_ModeWritingPrefab;


    /// <summary>
    /// Reference to prefab which writes "tool" on one side of the controller
    /// </summary>
    public GameObject m_ToolWritingPrefab;
    #endregion

    #region PRIVATE_VARIABLES
    /// <summary>
    /// Private SelectorTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private SelectorTool m_SelectorTool;

    /// <summary>
    /// Private ShootingTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private ShootingTool m_ShootingTool;

    /// <summary>
    /// Private TimeTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private TimeTool m_TimeTool;

    /// <summary>
    /// Private HandTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private HandTool m_HandTool;

    /// <summary>
    /// Private GUIController reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private GUIController m_GUIController;

    /// <summary>
    /// Private GUIController reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private ViewController m_ViewController;

    /// <summary>
    /// Private ModelSpawn reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private ModelSpawnController m_ModelSpawnController;

    /// <summary>
    /// Selection wheel to select tools.
    /// </summary>
    [SerializeField]
    private SelectionWheel m_ToolWheel;

    /// <summary>
    /// Selection wheel to select different GUI modes.
    /// </summary>
    [SerializeField]
    private SelectionWheel m_GUIWheel;

    /// <summary>
    /// Controllers initialized or not.
    /// </summary>
    private bool m_Initialized = false;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Calls the ray cast from the selector tool if it is active.
    /// </summary>
    void Update()
    {
        if (!m_Initialized)
        {
            return;
        }
        if (m_SelectorTool.gameObject.activeInHierarchy)
        {
            m_SelectorTool.GetRayFromController();

            //TODO Only for testin purposes-> delete!!!
            //ROSBridgeLib.custom_msgs.RoboyPoseMsg msg = new ROSBridgeLib.custom_msgs.RoboyPoseMsg("roboy", gameObject.name, transform.x, transform.y, transform.z, transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w );
            //ROSBridge.Instance.Publish(RoboyHandsPublisher.GetMessageTopic(), msg);

        }
        if (m_HandTool.gameObject.activeInHierarchy)
        {
            m_HandTool.CheckUserGrabbingRoboy();
        }

        //For the State Visualization UI: (VRUILogic is singleton so this is of no impact if not present)
        Valve.VR.EVRButtonId mask = Valve.VR.EVRButtonId.k_EButton_SteamVR_Touchpad;
        VRUILogic.Instance.SetTouchPosition(0, m_SelectorTool.Controller.GetAxis(mask));
        VRUILogic.Instance.SetTouchedInfo(0, m_SelectorTool.Controller.GetTouch(mask));
        if (m_GUIController)
        {
            VRUILogic.Instance.SetTouchPosition(1, m_GUIController.Controller.GetAxis(mask));
            VRUILogic.Instance.SetTouchedInfo(1, m_GUIController.Controller.GetTouch(mask));
        }
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Initialize all tools.
    /// </summary>
    public void Initialize(List<ControllerTool> toolList)
    {
        SetTools(toolList);
        StartCoroutine(initControllersCoroutine());
    }

    /// <summary>
    /// Changes view mode when the user presses the side button on the controller.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void OnChangeGUITool(object sender, ClickedEventArgs e)
    {
        if (m_GUIWheel != null)
        {
            m_GUIWheel.gameObject.SetActive(!m_GUIWheel.gameObject.activeSelf);
            m_ToolWheel.gameObject.SetActive(false);
            //TODO dirty / fast hack for now 
            //GameObject controller = m_GUIController.GetComponentInParent<SteamVR_TrackedController>().gameObject;
            //controller.SetActive(!m_GUIController.gameObject.activeSelf);
            //inform VRUILogic that GUI mode is currently being selected / not being selected anymore
            VRUILogic.Instance.SetGUIModeSelecting(m_GUIWheel.gameObject.activeSelf);
        }
        else if (m_ViewController != null && m_GUIController)
        {
            if (m_GUIController.gameObject.activeSelf)
            {
                m_GUIController.gameObject.SetActive(false);
                m_ViewController.gameObject.SetActive(true);
            }
            else if (m_ViewController.gameObject.activeSelf)
            {
                m_GUIController.gameObject.SetActive(true);
                m_ViewController.gameObject.SetActive(false);
            }
            //RoboyManager.Instance.ResetSimulation();
        }
    }

    /// <summary>
    /// Changes the tool when the user presses the side button on the controller.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void OnChangeTool(object sender, ClickedEventArgs e)
    {
        if (m_ToolWheel == null)
            ModeManager.Instance.ChangeToolMode();
        else
        { //activate or deactivate tool wheel depending on previous state
            bool previousState = m_ToolWheel.gameObject.activeSelf;
            m_ToolWheel.gameObject.SetActive(!previousState);
            m_GUIWheel.gameObject.SetActive(false);
            //TODO dirty / fast hack for now 
            //GameObject controller = m_ShootingTool.GetComponentInParent<SteamVR_TrackedController>().gameObject;
            //controller.SetActive(!m_GUIController.gameObject.activeSelf);
            VRUILogic.Instance.SetToolWheelState(!previousState);
        }
    }

    /// <summary>
    /// Retrives the touchpad input of the tool controller and updates the values.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void GetTouchpadInput(object sender, ClickedEventArgs e)
    {

        Vector2 touchPadPos = new Vector2(e.padX, e.padY);

        TouchpadStatus result = TouchpadStatus.None;

        if (Mathf.Abs(touchPadPos.x) < 0.4f && Mathf.Abs(touchPadPos.y) < 0.4f)
            return;

        if (Mathf.Abs(touchPadPos.x) > Mathf.Abs(touchPadPos.y))
        {
            if (touchPadPos.x > 0)
                result = TouchpadStatus.Right;
            else
                result = TouchpadStatus.Left;
        }
        else
        {
            if (touchPadPos.y > 0)
                result = TouchpadStatus.Top;
            else
                result = TouchpadStatus.Bottom;
        }

        if (e.controllerIndex.Equals(m_GUIController.Controller.index))
        {
            Debug.Log("[INPUTMANAGER] Check touchpad GUI controller");
            m_GUIController.CheckTouchPad(result);
        }

    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// Set all tools depending on their type to the respective variable.
    /// </summary>
    /// <param name="toolList"></param>
    private void SetTools(List<ControllerTool> toolList)
    {
        List<SelectionWheelPart> toolWheelParts = new List<SelectionWheelPart>();
        List<SelectionWheelPart> guiWheelParts = new List<SelectionWheelPart>();
        foreach (ControllerTool tool in toolList)
        {
            if (tool is GUIController)
            {
                m_GUIController = (GUIController)tool;
            }
            else if (tool is ModelSpawnController)
            {
                m_ModelSpawnController = (ModelSpawnController)tool;
            }
            else if (tool is SelectorTool)
            {
                m_SelectorTool = (SelectorTool)tool;
            }
            else if (tool is ShootingTool)
            {
                m_ShootingTool = (ShootingTool)tool;
            }
            else if (tool is TimeTool)
            {
                m_TimeTool = (TimeTool)tool;
            }
            else if (tool is ViewController)
            {
                m_ViewController = (ViewController)tool;
            }
            else if (tool is HandTool)
            {
                m_HandTool = (HandTool)tool;
            }

            if (m_ToolWheel)
            {
                SelectionWheelPart toolWheelPart;
                if ((toolWheelPart = tool.gameObject.GetComponent<SelectionWheelPartTool>()) != null)
                    toolWheelParts.Add(toolWheelPart);
            }
            if (m_GUIWheel)
            {
                SelectionWheelPart GUIWheelPart;
                if ((GUIWheelPart = tool.gameObject.GetComponent<SelectionWheelPartGUITool>()) != null)
                {
                    guiWheelParts.Add(GUIWheelPart);
                }
            }
        }

        if (m_ToolWheel)
        {
            m_ToolWheel.BindController(m_SelectorTool.ControllerObject);
            m_ToolWheel.Initialize(toolWheelParts, 0);
            m_ToolWheel.gameObject.SetActive(false);

            //add UI writing
            GameObject writing = Instantiate(m_ToolWritingPrefab, m_SelectorTool.ControllerObject.gameObject.transform);
            writing.transform.localPosition = m_ToolWritingPrefab.transform.position;
            //Debug.Log("[InputManager] Set Parent" + m_SelectorTool.name + " for 'tool'-writing");
            //TODO: for now set inactive, since different models make it harder to see
            writing.SetActive(false);
        }
        if (m_GUIWheel)
        {
            m_GUIWheel.BindController(m_GUIController.ControllerObject);
            m_GUIWheel.Initialize(guiWheelParts, 0);
            m_GUIWheel.gameObject.SetActive(false);
            //add UI writing
            GameObject writing = Instantiate(m_ModeWritingPrefab, m_GUIController.ControllerObject.gameObject.transform);
            //writing.transform.localPosition = m_ModeWritingPrefab.transform.position;
            Debug.Log("[InputManager] Set Parent " + m_GUIController.name + "for 'mode'-writing");
        }

    }

    /// <summary>
    /// Initializes all controllers and tools.
    /// </summary>
    /// <returns></returns>
    private IEnumerator initControllersCoroutine()
    {
        m_SelectorTool.gameObject.SetActive(true);
        m_ShootingTool.gameObject.SetActive(false);
        m_TimeTool.gameObject.SetActive(false);

        // right now the tool wheel is only in the RoboyInteractionScene, so we need to be aware of that
        if (m_ToolWheel)
        {
            m_HandTool.gameObject.SetActive(false);
        }

        SteamVR_RenderModel controllerModel = m_SelectorTool.transform.parent.GetComponentInChildren<SteamVR_RenderModel>();
        controllerModel.gameObject.SetActive(false);

        //If there is a view controller, disable it.
        if (m_ViewController != null)
        {
            m_ViewController.gameObject.SetActive(false);
        }
        //If there is a hand tool, disable it.
        if (m_HandTool != null)
        {
            m_HandTool.gameObject.SetActive(false);
        }
        while (m_SelectorTool.ControllerEventListener == null)
            yield return Time.fixedDeltaTime;

        m_SelectorTool.ControllerEventListener.PadClicked += GetTouchpadInput;
        if (m_GUIController)
        {
            m_GUIController.ControllerEventListener.PadClicked += GetTouchpadInput;
            m_GUIController.ControllerEventListener.Gripped += OnChangeGUITool;
        }
        if (m_ModelSpawnController)
        {
            m_ModelSpawnController.gameObject.SetActive(false);
        }

        //CHANGE THIS
        m_SelectorTool.ControllerEventListener.Gripped += OnChangeTool;
        //m_ShootingTool.ControllerEventListener.Gripped += ToolControllerSideButtons;

        m_Initialized = true;
    }
    #endregion
}
