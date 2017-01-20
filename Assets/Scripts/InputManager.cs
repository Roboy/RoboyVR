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

    }

    void Update()
    {
        if (m_SelectorTool.gameObject.activeInHierarchy)
        {
            m_SelectorTool.GetRayFromController();
            GetTouchpadInputs();
        }
    }

    void GetTouchpadInputs()
    {
        if (m_SelectorTool == null)
            return;

        string result = GetTouchpadInput(m_SelectorTool.SteamController);

        switch (result)
        {
            case "Right":
                SelectorTool_TouchpadStatus = TouchpadStatus.Right;
                break;
            case "Left":
                SelectorTool_TouchpadStatus = TouchpadStatus.Left;
                break;
            case "Top":
                SelectorTool_TouchpadStatus = TouchpadStatus.Top;
                break;
            case "Bottom":
                SelectorTool_TouchpadStatus = TouchpadStatus.Bottom;
                break;
            case "None":
                SelectorTool_TouchpadStatus = TouchpadStatus.None;
                break;
        }

        if (m_GUIController == null)
            return;

        result = GetTouchpadInput(m_GUIController.SteamController);

        switch (result)
        {
            case "Right":
                GUIController_TouchpadStatus = TouchpadStatus.Right;
                break;
            case "Left":
                GUIController_TouchpadStatus = TouchpadStatus.Left;
                break;
            case "Top":
                GUIController_TouchpadStatus = TouchpadStatus.Top;
                break;
            case "Bottom":
                GUIController_TouchpadStatus = TouchpadStatus.Bottom;
                break;
            case "None":
                GUIController_TouchpadStatus = TouchpadStatus.None;
                break;
        }

    }

    public string GetTouchpadInput(SteamVR_Controller.Device controller)
    {
        Vector2 touchPadPos = controller.GetAxis();
        string result = "None";

        //Debug.Log("X: " + touchPadPos.x + " Y: " + touchPadPos.y);

        if (Mathf.Abs(touchPadPos.x) < 0.4f && Mathf.Abs(touchPadPos.y) < 0.4f)
            return "None";

        if (Mathf.Abs(touchPadPos.x) > Mathf.Abs(touchPadPos.y))
        {
            if (touchPadPos.x > 0)
                result = "Right";
            else
                result = "Left";
        }
        else
        {
            if (touchPadPos.y > 0)
                result = "Top";
            else
                result = "Bottom";
        }

        return result;

    }
}
