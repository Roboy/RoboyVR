﻿using UnityEngine;
using UnityEngine.UI;
/// <summary>
/// Tool to switch between different views.
/// </summary>
public class ViewController : ControllerTool
{
    /// <summary>
    /// Reference to the SelectionPanel.
    /// </summary>
    [SerializeField]
    private GameObject m_Panel;

    [SerializeField]
    private Slider m_Slider;

    /// <summary>
    /// Initializes the controller variables.
    /// Intializes the UI Panels and creates them for every roboy part for every panel mode.
    /// </summary>
    void Start()
    {
        // get the selection panel
        //m_Panel = GetComponentInChildren<GameObject>();
        //m_Slider = m_Panel.GetComponent<Slider>();
        ViewSelectionManager.Instance.GazeboImage.GetComponent<CameraPanel>().Sl = m_Slider;
        ViewSelectionManager.Instance.HtcImage.GetComponent<CameraPanel>().Sl = m_Slider;
        ViewSelectionManager.Instance.ZedImage.GetComponent<CameraPanel>().Sl = m_Slider;
    }
}
