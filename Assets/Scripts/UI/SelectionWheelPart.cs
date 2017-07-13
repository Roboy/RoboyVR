using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectionWheelPart : MonoBehaviour {

    /// <summary>
    /// Highlight the part.
    /// </summary>
    public virtual void Highlight()
    {
        Debug.Log("Base class! No implementation!");
    }

    /// <summary>
    /// Unhighligh the part.
    /// </summary>
    public virtual void Unhighlight()
    {
        Debug.Log("Base class! No implementation!");
    }

    /// <summary>
    /// Select the part.
    /// </summary>
    public virtual void Select()
    {
        Debug.Log("Base class! No implementation!");
    }

    /// <summary>
    /// Deselect the part.
    /// </summary>
    public virtual void Deselect()
    {
        Debug.Log("Base class! No implementation!");
    }
}
