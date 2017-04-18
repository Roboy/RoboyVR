using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// InputManager holds a reference of every tool. On top of that it listens to button events from these tools and forwards touchpad input to the respective classes.
/// </summary>
public class InputManager : Singleton<InputManager> {

    /// <summary>
    /// Public GUIController reference.
    /// </summary>
    public GUIController GUI_Controller {
        get { return m_GUIController; }
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

    /// <summary>
    /// Private SelectorTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private SelectorTool m_SelectorTool;

    /// <summary>
    /// Private ShootingTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField] private ShootingTool m_ShootingTool;

    /// <summary>
    /// Private TimeTool reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private TimeTool m_TimeTool;

    /// <summary>
    /// Private GUIController reference. Is serialized so it can be dragged in the editor.
    /// </summary>
    [SerializeField]
    private GUIController m_GUIController;

    /// <summary>
    /// Touchpad status of the controller where selector tool is attached to.
    /// </summary>
    public TouchpadStatus SelectorTool_TouchpadStatus { get; private set; }

    /// <summary>
    /// Touchpad status of the controller where gui controller tool is attached to.
    /// </summary>
    public TouchpadStatus GUIController_TouchpadStatus { get; private set; }

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
    /// Initialize all tools.
    /// </summary>
    void Start()
    {
        StartCoroutine(InitControllers());
    }

    /// <summary>
    /// Calls the ray cast from the selector tool if it is active.
    /// </summary>
    void Update()
    {
        if (m_SelectorTool.gameObject.activeInHierarchy)
        {
            m_SelectorTool.GetRayFromController();
        }
    }

    /// <summary>
    /// Initializes all controllers and tools.
    /// </summary>
    /// <returns></returns>
    IEnumerator InitControllers()
    {
        m_SelectorTool.gameObject.SetActive(true);
        m_ShootingTool.gameObject.SetActive(false);
        m_TimeTool.gameObject.SetActive(false);

        while (m_SelectorTool.ControllerEventListener == null || m_GUIController.ControllerEventListener == null)
            yield return Time.fixedDeltaTime;

        m_SelectorTool.ControllerEventListener.PadClicked += GetTouchpadInput;
        m_GUIController.ControllerEventListener.PadClicked += GetTouchpadInput;

        m_GUIController.ControllerEventListener.Gripped += GUIControllerSideButtons;
        
        //CHANGE THIS
        m_SelectorTool.ControllerEventListener.Gripped += ToolControllerSideButtons;
        //m_ShootingTool.ControllerEventListener.Gripped += ToolControllerSideButtons;
    }

    /// <summary>
    /// Changes view mode when the user presses the side button on the controller.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void GUIControllerSideButtons(object sender, ClickedEventArgs e)
    {
        ModeManager.Instance.ChangeViewMode();
    }

    /// <summary>
    /// Changes the tool when the user presses the side button on the controller.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void ToolControllerSideButtons(object sender, ClickedEventArgs e)
    {
        ModeManager.Instance.ChangeToolMode();
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

        if (e.controllerIndex.Equals(m_SelectorTool.Controller.index))
        {
            SelectorTool_TouchpadStatus = result;
        }        
        else if (e.controllerIndex.Equals(m_GUIController.Controller.index))
        {         
            GUIController_TouchpadStatus = result;
            m_GUIController.CheckTouchPad(result);
        }
            
    }
}
