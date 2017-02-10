using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        Selection,
        Motorforce,
        MotorVoltage,
        MotorCurrent,
        EnergyConsumption,
        TendonForces
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
    /// Private variable for current view mode
    /// </summary>
    private Viewmode m_CurrentViewmode = Viewmode.Comparison;

    /// <summary>
    /// Private variable for current panel mode
    /// </summary>
    private Panelmode m_CurrentPanelmode = Panelmode.Selection;

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

    /// <summary>
    /// DONT USE THIS ONLY TEST
    /// </summary>
    private void testPanelModeChange()
    {
        m_CurrentPanelmode++;
    }
}
