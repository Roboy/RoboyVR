using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class ModeManager : Singleton<ModeManager> {

    /// <summary>
    /// We change between Single view where we can choose only one objet at a time and comparison view with three maximum objects at a time
    /// </summary>
    public enum Viewmode
    {
        Single,
        Comparison
    }

    /// <summary>
    /// Describes the different modes for panel visualization
    /// </summary>
    public enum Panelmode
    {
        Motor_Force,
        Motor_Voltage,
        Motor_Current,
        Energy_Consumption,
        Tendon_Forces
    }

    public enum GUIMode
    {
        Selection,
        GUIPanels
    }

    /// <summary>
    /// SelectorTool: Select roboy meshes;
    /// ShooterTool: Shoot projectiles at roboy;
    /// TimeTool: Reverse/stop time
    /// </summary>
    public enum ToolMode
    {
        SelectorTool,
        ShooterTool,
        TimeTool
    }

    /// <summary>
    /// Current view mode, READ ONLY
    /// </summary>
    public Viewmode CurrentViewmode {
        get { return m_CurrentViewmode; }
    }

    /// <summary>
    /// Current panel mode, READ ONLY
    /// </summary>
    public Panelmode CurrentPanelmode
    {
        get { return m_CurrentPanelmode; }
    }

    /// <summary>
    /// Current GUI mode, READ ONLY
    /// </summary>
    public GUIMode CurrentGUIMode
    {
        get { return m_CurrentGUIMode; }
    }

    /// <summary>
    /// Current Tool mode, READ ONLY
    /// </summary>
    public ToolMode CurrentToolMode
    {
        get { return m_CurrentToolMode;}
    }

    /// <summary>
    /// Private variable for current view mode
    /// </summary>
    private Viewmode m_CurrentViewmode = Viewmode.Comparison;

    /// <summary>
    /// Private variable for current panel mode
    /// </summary>
    private Panelmode m_CurrentPanelmode = Panelmode.Motor_Force;

    /// <summary>
    /// Private variable for current GUI mode
    /// </summary>
    private GUIMode m_CurrentGUIMode = GUIMode.Selection;

    /// <summary>
    /// Private variable for current Tool mode
    /// </summary>
    private ToolMode m_CurrentToolMode = ToolMode.SelectorTool;

    /// <summary>
    /// Changes between single and comparison view
    /// </summary>
    public void ChangeViewMode()
    {
        SelectorManager.Instance.ResetSelectedObjects();

        if (m_CurrentViewmode == Viewmode.Single)
        {
            m_CurrentViewmode = Viewmode.Comparison;
            SelectorManager.Instance.MaximumSelectableObjects = 3;
        }
        else if (m_CurrentViewmode == Viewmode.Comparison)
        {
            m_CurrentViewmode = Viewmode.Single;
            SelectorManager.Instance.MaximumSelectableObjects = 1;
        }  
    }

    public void ChangeGUIMode()
    {
        if (m_CurrentGUIMode == GUIMode.Selection)
        {
            m_CurrentGUIMode = GUIMode.GUIPanels;
        }
        else if (m_CurrentGUIMode == GUIMode.GUIPanels)
        {
            m_CurrentGUIMode = GUIMode.Selection;
        }
    }

    public void ChangeToolMode()
    {

        if (m_CurrentToolMode == ToolMode.SelectorTool)
        {
            //Debug.Log("toolmode to shoot");
            InputManager.Instance.Selector_Tool.enabled = false;
            InputManager.Instance.ShootingTool.enabled = true;
            InputManager.Instance.Selector_Tool.gameObject.SetActive(false);
            InputManager.Instance.ShootingTool.gameObject.SetActive(true);
            m_CurrentToolMode = ToolMode.ShooterTool;
        }
        else if (m_CurrentToolMode == ToolMode.ShooterTool)
        {
            //Debug.Log("toolmode to time");
            InputManager.Instance.TimeTool.enabled = true;
            InputManager.Instance.ShootingTool.enabled = false;
            InputManager.Instance.TimeTool.gameObject.SetActive(true);
            InputManager.Instance.ShootingTool.gameObject.SetActive(false);
            m_CurrentToolMode = ToolMode.TimeTool;
        }
        else if (m_CurrentToolMode == ToolMode.TimeTool)
        {
            //Debug.Log("toolmode to select");
            InputManager.Instance.Selector_Tool.enabled = true;
            InputManager.Instance.TimeTool.enabled = false;
            InputManager.Instance.Selector_Tool.gameObject.SetActive(true);
            InputManager.Instance.TimeTool.gameObject.SetActive(false);
            m_CurrentToolMode = ToolMode.SelectorTool;
        }
    }

    public void ChangePanelModeNext()
    {
        m_CurrentPanelmode =  (Panelmode)((int) (m_CurrentPanelmode+1) % Enum.GetNames(typeof(Panelmode)).Length);
    }

    public void ChangePanelModePrevious()
    {
        if (m_CurrentPanelmode == 0)
        {
            m_CurrentPanelmode = (Panelmode)(Enum.GetNames(typeof(Panelmode)).Length - 1);
        }
        else
        {
            m_CurrentPanelmode--;
        }     
    }

    public void ResetPanelMode()
    {
        m_CurrentPanelmode = Panelmode.Motor_Force;
    }

    /// <summary>
    /// DONT USE THIS ONLY TEST
    /// </summary>
    private void testPanelModeChange()
    {
        m_CurrentPanelmode++;
    }

}
