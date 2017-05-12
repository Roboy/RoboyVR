using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ControllerManager : Singleton<ControllerManager> {

    public Canvas InstructionCanvas;

    public Text GUIText;

    public Text ToolsText;

    public Image LeftBar;

    public Image RightBar;

    /// <summary>
    /// Prefab of the left controller.
    /// </summary>
    public SteamVR_TrackedObject ControllerForGUI;

    /// <summary>
    /// Prefab of the right controller.
    /// </summary>
    public SteamVR_TrackedObject ControllerForTools;
    
    /// <summary>
    /// List of tools that are bound to the left controller.
    /// </summary>
    public List<ControllerTool> LeftHandTools = new List<ControllerTool>();

    /// <summary>
    /// List of tools that are bound to the right controller.
    /// </summary>
    public List<ControllerTool> RightHandTools = new List<ControllerTool>();

    /// <summary>
    /// Check whether the left controller is set or not.
    /// </summary>
    [SerializeField]
    bool m_ControllerGUISet = false;

    /// <summary>
    /// Check whether the right controller is set or not.
    /// </summary>
    [SerializeField]
    bool m_ControllerToolsSet = false;

    /// <summary>
    /// Left controller input component.
    /// </summary>
    SteamVR_Controller.Device m_SteamVRDeviceLeft;

    /// <summary>
    /// Right controller input component.
    /// </summary>
    SteamVR_Controller.Device m_SteamVRDeviceRight;

    private void Start()
    {
        StartCoroutine(checkForControllersCoroutine());   
    }

    /// <summary>
    /// Check and assign tools accordingly.
    /// </summary>
    /// <returns></returns>
    IEnumerator checkForControllersCoroutine()
    {
        // Check whether the controllers are valid or not and assign the input components.
        while (!ControllerForGUI.isValid)
            yield return null;

        m_SteamVRDeviceLeft = SteamVR_Controller.Input((int)ControllerForGUI.index);

        while (!ControllerForTools.isValid)
            yield return null;

        m_SteamVRDeviceRight = SteamVR_Controller.Input((int)ControllerForTools.index);

        List<ControllerTool> toolList = new List<ControllerTool>();

        bool leftControllerSet = false;
        bool rightcontrollerSet = false;

        // Wait until both controllers are set
        while (!m_ControllerGUISet || !m_ControllerToolsSet)
        {
            // let the controllers vibrate on input
            if (m_SteamVRDeviceLeft.GetHairTrigger())
            {
                m_SteamVRDeviceLeft.TriggerHapticPulse(500);
                LeftBar.gameObject.SetActive(true);
                LeftBar.transform.localScale = new Vector3(1, m_SteamVRDeviceLeft.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x, 1);
            }
            else
            {
                LeftBar.gameObject.SetActive(false);
            }
            if (m_SteamVRDeviceRight.GetHairTrigger())
            {
                m_SteamVRDeviceRight.TriggerHapticPulse(500);
                RightBar.gameObject.SetActive(true);
                RightBar.transform.localScale = new Vector3(1, m_SteamVRDeviceRight.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x, 1);
            }
            else
            {
                RightBar.gameObject.SetActive(false);
            }

            // Check for left controller
            if (m_SteamVRDeviceLeft.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x > 0.99f && !leftControllerSet)
            {
                // if the left controller is not set yet
                if (!m_ControllerGUISet)
                {
                    // then assign all tools from the left hand to the left controller
                    foreach (ControllerTool t in LeftHandTools)
                    {
                        toolList.Add(Instantiate(t, ControllerForGUI.transform));
                    }
                    m_ControllerGUISet = true;
                    leftControllerSet = true;
                    GUIText.gameObject.SetActive(false);
                    ToolsText.gameObject.SetActive(true);
                }
                // otherwise assin all tools from the right hand to the left controller
                else
                {
                    foreach (ControllerTool t in RightHandTools)
                    {
                        toolList.Add(Instantiate(t, ControllerForGUI.transform));
                    }
                    m_ControllerToolsSet = true;
                    leftControllerSet = true;
                    InstructionCanvas.gameObject.SetActive(false);
                }

            }
            // Check for right controller
            if (m_SteamVRDeviceRight.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x > 0.99f && !rightcontrollerSet)
            {
                // if the left controller is not set yet
                if (!m_ControllerGUISet)
                {
                    // then assign all tools from the left hand to the right controller
                    foreach (ControllerTool t in LeftHandTools)
                    {
                        toolList.Add(Instantiate(t, ControllerForTools.transform));
                    }
                    m_ControllerGUISet = true;
                    rightcontrollerSet = true;
                    GUIText.gameObject.SetActive(false);
                    ToolsText.gameObject.SetActive(true);
                }
                // otherwise assin all tools from the right hand to the right controller
                else
                {
                    foreach (ControllerTool t in RightHandTools)
                    {
                        toolList.Add(Instantiate(t, ControllerForTools.transform));
                    }
                    m_ControllerToolsSet = true;
                    rightcontrollerSet = true;
                    InstructionCanvas.gameObject.SetActive(false);
                }
            }

            if(leftControllerSet)
                LeftBar.gameObject.SetActive(false);
            if(rightcontrollerSet)
                RightBar.gameObject.SetActive(false);

            yield return null;
        }

        InputManager.Instance.Initialize(toolList);
    }
}
