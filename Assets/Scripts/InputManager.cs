using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputManager : Singleton<InputManager> {

    public GUIController GUI_Controller {
        get { return m_GUIController; }
    }

    public SelectorTool Selector_Tool
    {
        get { return m_SelectorTool; }
    }

    public ShootingTool ShootingTool
    {
        get { return m_ShootingTool; }
    }

    public TimeTool TimeTool
    {
        get { return m_TimeTool; }
    }

    [SerializeField]
    private SelectorTool m_SelectorTool;

    [SerializeField] private ShootingTool m_ShootingTool;

    [SerializeField]
    private TimeTool m_TimeTool;

    [SerializeField]
    private GUIController m_GUIController;

    public TouchpadStatus SelectorTool_TouchpadStatus { get; private set; }
    public TouchpadStatus GUIController_TouchpadStatus { get; private set; }

    public enum TouchpadStatus
    {
        Right,
        Left,
        Top,
        Bottom,
        None
    }
    
    void Start()
    {
        StartCoroutine(InitControllers());
    }

    void Update()
    {
        if (m_SelectorTool.gameObject.activeInHierarchy)
        {
            m_SelectorTool.GetRayFromController();
        }

       
    }

    IEnumerator InitControllers()
    {
        while (m_SelectorTool.ControllerEventListener == null || m_GUIController.ControllerEventListener == null)
            yield return Time.fixedDeltaTime;

        m_SelectorTool.ControllerEventListener.PadClicked += GetTouchpadInput;
        m_GUIController.ControllerEventListener.PadClicked += GetTouchpadInput;

        m_GUIController.ControllerEventListener.Gripped += GUIControllerSideButtons;
        
        //CHANGE THIS
        m_SelectorTool.ControllerEventListener.Gripped += ToolControllerSideButtons;
        //m_ShootingTool.ControllerEventListener.Gripped += ToolControllerSideButtons;
    }

    public void GUIControllerSideButtons(object sender, ClickedEventArgs e)
    {
        ModeManager.Instance.ChangeViewMode();
    }

    public void ToolControllerSideButtons(object sender, ClickedEventArgs e)
    {
        ModeManager.Instance.ChangeToolMode();
    }

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
