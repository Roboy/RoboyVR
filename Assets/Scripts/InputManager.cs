using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputManager : Singleton<InputManager> {

    [SerializeField]
    private SelectorTool m_SelectorTool;

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
    }


    public void GetTouchpadInput(object sender, ClickedEventArgs e)
    {
        Vector2 touchPadPos = new Vector2(e.padX, e.padY);

        TouchpadStatus result = TouchpadStatus.None;

        //Debug.Log("X: " + touchPadPos.x + " Y: " + touchPadPos.y);

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
            Debug.Log("Select : " + result);
            SelectorTool_TouchpadStatus = result;
        }        
        else if (e.controllerIndex.Equals(m_GUIController.Controller.index))
        {
            Debug.Log("GUI : " + result);           
            GUIController_TouchpadStatus = result;
            StartCoroutine(m_GUIController.ChangePanel());
        }
            
    }
}
