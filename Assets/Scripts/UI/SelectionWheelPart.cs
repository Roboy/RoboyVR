using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SelectionWheelPart : MonoBehaviour {

    public Sprite Icon;

    /// <summary>
    /// We use an intermediate function so you can override the getUIPart function to return the inherited part.
    /// </summary>
    public SelectionWheelPartUI UIPart { get { return getUIPart(); } } 

    /// <summary>
    /// Connection back to the UI part so both parts have a reference to each other.
    /// </summary>
    private SelectionWheelPartUI m_UIPart;

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

    /// <summary>
    /// Creates a reference to the UI part.
    /// </summary>
    public virtual void Initialize(SelectionWheelPartUI uiPart)
    {
        m_UIPart = uiPart;
    }

    protected virtual SelectionWheelPartUI getUIPart()
    {
        return m_UIPart; 
    }
}
