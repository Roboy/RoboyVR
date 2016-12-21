using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SelectableObject : MonoBehaviour
{

    #region PUBLIC_VARIABLES
    public enum State { DEFAULT, TARGETED, SELECTED }

    public State CurrentState
    {
        get { return m_CurrentState; }
    }

    public Material TargetedMaterial;
    public Material SelectedMaterial;
    #endregion //PUBLIC_VARIABLES

    #region PRIVATE_VARIABLES
    private State m_CurrentState = State.DEFAULT;

    private Renderer[] m_Renderers;

    private Material m_DefaultMaterial;

    #endregion //PRIVATE_VARIABLES

    #region MONOBEHAVIOR_METHODS

    void Awake()
    {
        m_Renderers = GetComponentsInChildren<Renderer>();
        m_DefaultMaterial = m_Renderers[0].material;
    }

    void OnMouseDown()
    {
        SetStateSelected();
    }

    void OnMouseEnter()
    {
        SetStateTargeted();
    }

    void OnMouseExit()
    {
        SetStateDefault();
    }

    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

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

    public void SetStateTargeted()
    {
        if (m_CurrentState == State.DEFAULT)
        {
            changeState(State.TARGETED);
        }
    }

    public void SetStateDefault()
    {
        if (m_CurrentState == State.TARGETED)
        {
            changeState(State.DEFAULT);
        }
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    private void changeState(State s)
    {
        m_CurrentState = s;
        switch (m_CurrentState)
        {
            case State.DEFAULT:
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = m_DefaultMaterial;
                }
                break;
            case State.TARGETED:
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = TargetedMaterial;
                }
                break;
            case State.SELECTED:
                foreach (var mRenderer in m_Renderers)
                {
                    mRenderer.material = SelectedMaterial;
                }
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    #endregion //PRIVATE_METHODS

}
