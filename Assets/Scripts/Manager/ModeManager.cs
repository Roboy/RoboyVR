using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
/// ModeManager holds a reference of every active mode and provides function to switch between them.
/// This includes:
/// - Current tool: ShootingTool, SelectionTool etc.
/// - Current view mode: singe vs. comparison
/// - Current GUI mode: selection vs. GUI panels
/// - Current panel mode: motorforce, motorvoltage etc.
/// </summary>
public class ModeManager : Singleton<ModeManager> {

    /// <summary>
    /// We change between Single view where we can choose only one objet at a time and comparison view with three maximum objects at a time.
    /// </summary>
    public enum Viewmode
    {
        Single,
        Comparison
    }

    /// <summary>
    /// Describes the different modes for panel visualization.
    /// </summary>
    public enum Panelmode
    {
        Motor_Force,
        Motor_Voltage,
        Motor_Current,
        Energy_Consumption,
        Tendon_Forces
    }

    /// <summary>
    /// Enum for current GUI mode.
    /// </summary>
    public enum GUIMode
    {
        Selection,
        GUIPanels
    }

    /// <summary>
    /// SelectorTool: Select roboy meshes.
    /// ShooterTool: Shoot projectiles at roboy.
    /// TimeTool: Reverse/stop time.
    /// </summary>
    public enum ToolMode
    {
        SelectorTool,
        ShootingTool,
        TimeTool,
        HandTool,
        Undefined
    }

    /// <summary>
    /// Current view mode, READ ONLY.
    /// </summary>
    public Viewmode CurrentViewmode {
        get { return m_CurrentViewmode; }
    }

    /// <summary>
    /// Current panel mode, READ ONLY.
    /// </summary>
    public Panelmode CurrentPanelmode
    {
        get { return m_CurrentPanelmode; }
    }

    /// <summary>
    /// Current GUI mode, READ ONLY.
    /// </summary>
    public GUIMode CurrentGUIMode
    {
        get { return m_CurrentGUIMode; }
    }

    /// <summary>
    /// Current Tool mode, READ ONLY.
    /// </summary>
    public ToolMode CurrentToolMode
    {
        get { return m_CurrentToolMode;}
    }

    /// <summary>
    /// Private variable for current view mode.
    /// </summary>
    private Viewmode m_CurrentViewmode = Viewmode.Comparison;

    /// <summary>
    /// Private variable for current panel mode.
    /// </summary>
    private Panelmode m_CurrentPanelmode = Panelmode.Motor_Force;

    /// <summary>
    /// Private variable for current GUI mode.
    /// </summary>
    private GUIMode m_CurrentGUIMode = GUIMode.Selection;

    /// <summary>
    /// Private variable for current Tool mode.
    /// </summary>
    private ToolMode m_CurrentToolMode = ToolMode.SelectorTool;

    /// <summary>
    /// Changes between single and comparison view.
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

    /// <summary>
    /// Switches between selection and panels GUI mode.
    /// </summary>
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

    /// <summary>
    /// Switches between all tools.
    /// </summary>
    public void ChangeToolMode()
    {

        if (m_CurrentToolMode == ToolMode.SelectorTool)
        {
            //Debug.Log("toolmode to shoot");
            InputManager.Instance.Selector_Tool.enabled = false;
            InputManager.Instance.ShootingTool.enabled = true;
            InputManager.Instance.Selector_Tool.gameObject.SetActive(false);
            InputManager.Instance.ShootingTool.gameObject.SetActive(true);
            m_CurrentToolMode = ToolMode.ShootingTool;
        }
        else if (m_CurrentToolMode == ToolMode.ShootingTool)
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
            //Debug.Log("toolmode to hand");
            InputManager.Instance.HandTool.enabled = true;
            InputManager.Instance.TimeTool.enabled = false;
            InputManager.Instance.HandTool.gameObject.SetActive(true);
            InputManager.Instance.TimeTool.gameObject.SetActive(false);
            m_CurrentToolMode = ToolMode.HandTool;
        }
        else if (m_CurrentToolMode == ToolMode.HandTool)
        {
            //Debug.Log("toolmode to select");
            InputManager.Instance.Selector_Tool.enabled = true;
            InputManager.Instance.HandTool.enabled = false;
            InputManager.Instance.Selector_Tool.gameObject.SetActive(true);
            InputManager.Instance.HandTool.gameObject.SetActive(false);
            m_CurrentToolMode = ToolMode.SelectorTool;
        }

    }

    /// <summary>
    /// Changes the tool mode based on the enum to the new one and turns off the old tool.
    /// </summary>
    /// <param name="mode"></param>
    public void ChangeToolMode(ToolMode mode)
    {
        changeToolStatus(m_CurrentToolMode, false);
        m_CurrentToolMode = mode;
        changeToolStatus(m_CurrentToolMode, true);
    }

    public void ChangeToolMode(ControllerTool tool)
    {
        changeToolStatus(m_CurrentToolMode, false);
        m_CurrentToolMode = mapToolTypeToEnum(tool);
        changeToolStatus(m_CurrentToolMode, true);
    }

    /// <summary>
    /// Changes the panel mode to the next one based on the order in the enum defintion.
    /// </summary>
    public void ChangePanelModeNext()
    {
        m_CurrentPanelmode =  (Panelmode)((int) (m_CurrentPanelmode+1) % Enum.GetNames(typeof(Panelmode)).Length);
    }

    /// <summary>
    /// Changes the panel mode to the previous one based on the order in the enum defintion.
    /// </summary>
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

    /// <summary>
    /// Resets current panel mode to MotorForce.
    /// </summary>
    public void ResetPanelMode()
    {
        m_CurrentPanelmode = Panelmode.Motor_Force;
    }

    /// <summary>
    /// Changes the tool based on the enum to the new state.
    /// </summary>
    /// <param name="tool"></param>
    /// <param name="state"></param>
    private void changeToolStatus(ToolMode tool, bool state)
    {
        switch (tool)
        {
            case ToolMode.SelectorTool:
                InputManager.Instance.Selector_Tool.enabled = state;
                InputManager.Instance.Selector_Tool.gameObject.SetActive(state);
                break;
            case ToolMode.ShootingTool:
                InputManager.Instance.ShootingTool.enabled = state;
                InputManager.Instance.ShootingTool.gameObject.SetActive(state);
                break;
            case ToolMode.TimeTool:
                InputManager.Instance.TimeTool.enabled = state;
                InputManager.Instance.TimeTool.gameObject.SetActive(state);
                break;
            case ToolMode.HandTool:
                InputManager.Instance.HandTool.enabled = state;
                InputManager.Instance.HandTool.gameObject.SetActive(state);
                break;
            default:
                Debug.Log("Tool mode: " + tool + " not implemented!");
                break;
        }
    }

    private ToolMode mapToolTypeToEnum(ControllerTool tool)
    {
        if (tool is SelectorTool)
        {
            return ToolMode.SelectorTool;
        }
        else if (tool is ShootingTool)
        {
            return ToolMode.ShootingTool;
        }
        else if (tool is TimeTool)
        {
            return ToolMode.TimeTool;
        }
        else if (tool is HandTool)
        {
            return ToolMode.HandTool;
        }
        else
        {
            Debug.Log("Tool mode: " + tool + " not implemented!");
            return ToolMode.Undefined;
        }
    }
}
