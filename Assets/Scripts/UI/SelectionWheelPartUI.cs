using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Base class for parts in the selection wheel. It is expected that you extend the functionality
/// by creating a class which derives from this. All functions are marked as virtual and variables as protected.
/// </summary>
[RequireComponent(typeof(Image))]
[RequireComponent(typeof(Outline))]
public class SelectionWheelPartUI : MonoBehaviour {

    /// <summary>
    /// Color when the part is selected.
    /// </summary>
    public Color SelectedColor;

    /// <summary>
    /// Color when the part is highlighted.
    /// </summary>
    public Color HighlightColor;

    /// <summary>
    /// Color when the part is neither highlighter nor selected.
    /// </summary>
    public Color DefaultColor;

    /// <summary>
    /// Default outline color.
    /// </summary>
    public Color DefaultOutlineColor;

    /// <summary>
    /// Selected outline color.
    /// </summary>
    public Color SelectedOutlineColor;

    /// <summary>
    /// Image component.
    /// </summary>
    protected Image m_Image;

    /// <summary>
    /// Outline component to show if the part is selected.
    /// </summary>
    protected Outline m_Outline;



    /// <summary>
    /// Possible states of the part.
    /// </summary>
    protected enum State
    {
        Unhighlighted,
        Highlighted,
        Selected
    }

    /// <summary>
    /// Current state.
    /// </summary>
    protected State m_CurrentState = State.Unhighlighted;

    /// <summary>
    /// Returns the actual part. Override this property in your actual class so it returns the part you implemented.
    /// </summary>
    public SelectionWheelPart m_SelectionWheelPart { get { return getPart(); } }

    /// <summary>
    /// The actual content of this part.
    /// </summary>
    private SelectionWheelPart m_ActualPart;

    /// <summary>
    /// Get all components and initialze the colors.
    /// </summary>
    protected void Awake()
    {
        m_Image = GetComponent<Image>();
        m_Outline = GetComponent<Outline>();
        m_Outline.effectColor = DefaultOutlineColor;
        m_Outline.effectDistance = new Vector2(5f, -5f);
        m_Outline.enabled = false;
        m_Outline.enabled = true;
    }

    /// <summary>
    /// Initializes the actual part.
    /// </summary>
    /// <param name="wheelPart"></param>
    public virtual void Initialize(SelectionWheelPart wheelPart)
    {
        m_ActualPart = wheelPart;
        m_ActualPart.Initialize(this);
    }

    public void CreateIcon(Vector3 position, float zAngle)
    {
        GameObject iconGO = new GameObject("Icon");
        iconGO.transform.parent = transform;
        //iconGO.transform.localScale = Vector3.one;
        iconGO.transform.localPosition = position;
        iconGO.transform.localEulerAngles = new Vector3(0, 0, zAngle);
        iconGO.transform.localScale = transform.localScale * 0.8f;
        Image iconImg = iconGO.AddComponent<Image>();
        iconImg.sprite = m_ActualPart.Icon;
    }

    /// <summary>
    /// Highlight the part.
    /// </summary>
    public virtual void Highlight()
    {
        if (m_CurrentState != State.Unhighlighted)
            return;

        m_Image.color = HighlightColor;
        m_ActualPart.Highlight();
        m_CurrentState = State.Highlighted;
    }

    /// <summary>
    /// Unhighligh the part.
    /// </summary>
    public virtual void Unhighlight()
    {
        if (m_CurrentState == State.Selected)
            return;

        m_Image.color = DefaultColor;
        m_ActualPart.Unhighlight();
        m_CurrentState = State.Unhighlighted;
    }

    /// <summary>
    /// Select the part.
    /// </summary>
    public virtual void Select()
    {
        m_Image.color = SelectedColor;
        m_Outline.effectColor = SelectedOutlineColor;
        m_ActualPart.Select();
        m_CurrentState = State.Selected;
    }

    /// <summary>
    /// Deselect the part.
    /// </summary>
    public virtual void Deselect()
    {
        if (m_CurrentState != State.Selected)
            return;

        m_Image.color = DefaultColor;
        m_Outline.effectColor = DefaultOutlineColor;
        m_ActualPart.Deselect();
        m_CurrentState = State.Unhighlighted;
    }

    protected virtual SelectionWheelPart getPart()
    {
        return m_ActualPart;
    }
}
