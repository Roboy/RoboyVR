using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectionWheelPartGUITool : SelectionWheelPart {

    public ControllerTool Tool
    {
        get
        {
            if (m_Tool == null)
                m_Tool = GetComponent<ControllerTool>();
            return m_Tool;
        }
    }

    private ControllerTool m_Tool;

    /// <summary>
    /// Highlight the part.
    /// </summary>
    public override void Highlight()
    {
        //Debug.Log(gameObject.name + " : Highlight!");
    }

    /// <summary>
    /// Unhighlight the part.
    /// </summary>
    public override void Unhighlight()
    {
        //Debug.Log(gameObject.name + " : Unhighlight!");
    }

    /// <summary>
    /// Select the part.
    /// </summary>
    public override void Select()
    {
        ModeManager.Instance.ChangeGUIToolMode(Tool);
    }

    /// <summary>
    /// Deselect the part.
    /// </summary>
    public override void Deselect()
    {
        //Debug.Log(gameObject.name + " : Deselect!");
    }
}
