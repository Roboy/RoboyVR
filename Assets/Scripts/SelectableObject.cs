using System;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// SelectableObject is attached on every roboy part. Is used to switch between selection states, which then again changes the material and manages GUI highlighting.
/// </summary>
public class SelectableObject : MonoBehaviour
{

    #region PUBLIC_VARIABLES
    /// <summary>
    /// Enum for possible selection states.
    /// </summary>
    public enum State { DEFAULT, TARGETED, SELECTED }

    /// <summary>
    /// Public property to track the selection state for outside classes.
    /// </summary>
    public State CurrentState
    {
        get { return m_CurrentState; }
    }

    /// <summary>
    /// Material of meshes which are targeted.
    /// </summary>
    public Material TargetedMaterial;

    /// <summary>
    /// Material of meshes which are selected.
    /// </summary>
    public Material SelectedMaterial;
    #endregion //PUBLIC_VARIABLES

    #region PRIVATE_VARIABLES

    /// <summary>
    /// Variable to track the current selection state.
    /// </summary>
    private State m_CurrentState = State.DEFAULT;

    /// <summary>
    /// Array of all renderer to change the material.
    /// </summary>
    private Renderer[] m_Renderers;

    /// <summary>
    /// Default material of all meshes.
    /// </summary>
    private Material m_DefaultMaterial;

    #endregion //PRIVATE_VARIABLES

    #region MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initializes the renderer array and default material.
    /// </summary>
    void Awake()
    {
        m_Renderers = GetComponentsInChildren<Renderer>();
        m_DefaultMaterial = m_Renderers[0].material;
    }
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    /// <summary>
    /// Changes the state depending on the current state and updates the result in SelectorManager.
    /// </summary>
    public void SetStateSelected()
    {
        if (m_CurrentState == State.TARGETED)
        {
            changeState(State.SELECTED);
            SelectorManager.Instance.AddSelectedObject(this);
        }
        else if (m_CurrentState == State.SELECTED)
        {
            changeState(State.TARGETED);
            SelectorManager.Instance.RemoveSelectedObject(this);
        }
    }

    /// <summary>
    /// Sets the state to targeted if the last state was default.
    /// </summary>
    public void SetStateTargeted()
    {
        if (m_CurrentState == State.DEFAULT)
        {
            changeState(State.TARGETED);
        }
    }

    /// <summary>
    /// Resets the state to default if the last state was targeted (without force mode).
    /// </summary>
    /// <param name="forceMode">Boolean to force the state switch.</param>
    public void SetStateDefault(bool forceMode = false)
    {
        if (forceMode)
            changeState(State.DEFAULT);

        if (m_CurrentState == State.TARGETED)
        {
            changeState(State.DEFAULT);
        }
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    /// <summary>
    /// Switches the state based on the parameter and manages GUI highlighting.
    /// </summary>
    /// <param name="s">State to which the object should switch to.</param>
    private void changeState(State s)
    {
        m_CurrentState = s;

        GameObject obj = null;
        if ( SelectorManager.Instance.UI_Elements != null && SelectorManager.Instance.UI_Elements.ContainsKey(name))
        {
            obj = SelectorManager.Instance.UI_Elements[name];
        }
        switch (m_CurrentState)
        {
            // Reset the objects material and the corrensponding GUI component
            case State.DEFAULT:
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = m_DefaultMaterial;
                }
                if (obj)
                {
                    obj.GetComponent<Outline>().enabled = false;
                }
                break;
            case State.TARGETED:
                // Set the objects material to targeted and highlight the corresponding GUI component with color of targeted material
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = TargetedMaterial;
                }
                if (obj)
                {
                    obj.GetComponent<Outline>().enabled = true;
                    obj.GetComponent<Outline>().effectColor = TargetedMaterial.color;
                }
                break;
            case State.SELECTED:
                // Set the objects material to selected and highlight the corresponding GUI component with color of selected material
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = SelectedMaterial;
                }
                if (obj)
                    obj.GetComponent<Outline>().effectColor = SelectedMaterial.color;
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    #endregion //PRIVATE_METHODS

}
