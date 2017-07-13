using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// The actual implementation for the selection wheel part tool.
/// </summary>
public class SelectionWheelPartTool : SelectionWheelPart {

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
        //Debug.Log(gameObject.name + " : Select!");
    }

    /// <summary>
    /// Deselect the part.
    /// </summary>
    public override void Deselect()
    {
        //Debug.Log(gameObject.name + " : Deselect!");
    }
}
