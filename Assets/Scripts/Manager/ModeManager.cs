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
public class ModeManager : Singleton<ModeManager>
{
    #region PUBLIC_VARIABLES
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
    public enum GUIViewerMode
    {
        /// <summary>
        /// Panel of all selectable mesh parts of the current roboy model
        /// </summary>
        Selection,
        /// <summary>
        /// Graphs for each motor for each type of information
        /// </summary>
        MotorValues
    }

    public enum SpawnViewerMode
    {
        /// <summary>
        /// Insert mode to add a preview model
        /// </summary>
        Insert,
        /// <summary>
        /// Delete a model via pointer
        /// </summary>
        Remove,
    }

    public enum GUIMode
    {
        GUIViewer,
        BeRoboyViewer,
        SpawnViewer,
        Undefined
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
    public Viewmode CurrentViewmode
    {
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
    /// Current GUIViewer mode, READ ONLY.
    /// </summary>
    public GUIViewerMode CurrentGUIViewerMode
    {
        get { return m_CurrentGUIViewerMode; }
    }

    public SpawnViewerMode CurrentSpawnViewerMode
    {
        get { return m_CurrentSpawnViewerMode; }
        set { m_CurrentSpawnViewerMode = value; }
    }

    /// <summary>
    /// Current Tool mode, READ ONLY.
    /// </summary>
    public ToolMode CurrentToolMode
    {
        get { return m_CurrentToolMode; }
    }

    /// <summary>
    /// Current GUI Mode. READ ONLY.
    /// </summary>
    public GUIMode CurrentGUIMode
    {
        get { return m_CurrentGUIMode; }
    }
    #endregion

    #region PRIVATE_VARIABLES
    /// <summary>
    /// Private variable for current view mode.
    /// </summary>
    private Viewmode m_CurrentViewmode = Viewmode.Comparison;

    /// <summary>
    /// Private variable for current panel mode.
    /// </summary>
    private Panelmode m_CurrentPanelmode = Panelmode.Motor_Force;

    /// <summary>
    /// Private variable for current GUIViewer mode.
    /// </summary>
    private GUIViewerMode m_CurrentGUIViewerMode = GUIViewerMode.Selection;

    /// <summary>
    /// Private variable for the current mode of the model spawn controller.
    /// </summary>
    private SpawnViewerMode m_CurrentSpawnViewerMode = SpawnViewerMode.Insert;

    /// <summary>
    /// Private variable for current Tool mode.
    /// </summary>
    private ToolMode m_CurrentToolMode = ToolMode.SelectorTool;

    /// <summary>
    /// Current view mode of the GUI tools
    /// </summary>
    private GUIMode m_CurrentGUIMode = GUIMode.GUIViewer;

    /// <summary>
    /// Holds reference to currently active controller tool from inputmanager
    /// </summary>
    private ControllerTool m_CurrentlyActiveTool;
    #endregion

    #region PUBLIC_METHODS
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
    public void ChangeGUIViewerMode()
    {
        if (m_CurrentGUIViewerMode == GUIViewerMode.Selection)
        {
            m_CurrentGUIViewerMode = GUIViewerMode.MotorValues;
        }
        else if (m_CurrentGUIViewerMode == GUIViewerMode.MotorValues)
        {
            m_CurrentGUIViewerMode = GUIViewerMode.Selection;
        }
    }

    /// <summary>
    /// Switches between all tools.
    /// Deactivates previous tool and activates new one
    /// </summary>
    public void ChangeToolMode()
    {
        switch (m_CurrentToolMode)
        {
            case ToolMode.SelectorTool:
                SetNewActiveTool(InputManager.Instance.ShootingTool);
                m_CurrentToolMode = ToolMode.ShootingTool;
                break;
            case ToolMode.ShootingTool:
                SetNewActiveTool(InputManager.Instance.TimeTool);
                m_CurrentToolMode = ToolMode.TimeTool;
                break;
            case ToolMode.TimeTool:
                SetNewActiveTool(InputManager.Instance.HandTool);
                m_CurrentToolMode = ToolMode.HandTool;
                break;
            case ToolMode.HandTool:
                SetNewActiveTool(InputManager.Instance.Selector_Tool);
                m_CurrentToolMode = ToolMode.SelectorTool;
                break;
            case ToolMode.Undefined:
                return;
            default:
                return;
        }
    }

    public void ChangeToolMode(ControllerTool tool)
    {
        m_CurrentToolMode = mapToolTypeToEnum(tool);

        //update controller object
        if (m_CurrentlyActiveTool)
        {
            m_CurrentlyActiveTool.EndTool();
            m_CurrentlyActiveTool.enabled = false;
            m_CurrentlyActiveTool.gameObject.SetActive(false);
        }


        tool.enabled = true;
        tool.gameObject.SetActive(true);
        m_CurrentlyActiveTool = tool;
        //update tool mode in case this tool is active, ignore  otherwise
        VRUILogic.Instance.SetToolMode(m_CurrentToolMode);

    }

    public void ChangeGUIToolMode(ControllerTool tool)
    {
        changeGUIToolStatus(m_CurrentGUIMode, false);
        m_CurrentGUIMode = mapGUIToolToEnum(tool);
        changeGUIToolStatus(m_CurrentGUIMode, true);
    }

    /// <summary>
    /// Changes the panel mode to the next one based on the order in the enum defintion.
    /// </summary>
    public void ChangePanelModeNext()
    {
        m_CurrentPanelmode = (Panelmode)((int)(m_CurrentPanelmode + 1) % Enum.GetNames(typeof(Panelmode)).Length);
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
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// deactivates currently active tool and activates new tool
    /// </summary>
    /// <param name="tool"></param>
    private void SetNewActiveTool(ControllerTool tool)
    {
        m_CurrentlyActiveTool.EndTool();
        m_CurrentlyActiveTool.enabled = false;
        m_CurrentlyActiveTool.gameObject.SetActive(false);
        m_CurrentlyActiveTool = tool;
        m_CurrentlyActiveTool.enabled = true;
        m_CurrentlyActiveTool.gameObject.SetActive(true);
    }

    private void changeGUIToolStatus(GUIMode mode, bool state)
    {
        switch (mode)
        {
            case GUIMode.GUIViewer:
                InputManager.Instance.GUI_Controller.enabled = state;
                InputManager.Instance.GUI_Controller.gameObject.SetActive(state);
                break;
            case GUIMode.BeRoboyViewer:
                InputManager.Instance.View_Controller.enabled = state;
                InputManager.Instance.View_Controller.gameObject.SetActive(state);
                break;
            case GUIMode.SpawnViewer:
                InputManager.Instance.ModelSpawn_Controller.enabled = state;
                InputManager.Instance.ModelSpawn_Controller.gameObject.SetActive(state);
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

    private GUIMode mapGUIToolToEnum(ControllerTool tool)
    {
        if (tool is GUIController)
        {
            return GUIMode.GUIViewer;
        }
        else if (tool is ViewController)
        {
            return GUIMode.BeRoboyViewer;
        }
        else if (tool is ModelSpawnController)
        {
            return GUIMode.SpawnViewer;
        }
        else
        {
            Debug.Log("GUI mode: " + tool + " not implemented!");
            return GUIMode.Undefined;
        }
    }
    #endregion
}
