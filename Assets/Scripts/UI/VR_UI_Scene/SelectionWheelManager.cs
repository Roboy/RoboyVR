﻿using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Implements a selectionwheel using the child objects (with RectTransforms) as options.
/// It enables and disables the respective Canvas depending on user input.
/// It displays a selection wheel, checks for user input and updates the current selection.
/// </summary>
public class SelectionWheelManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES

    // PUBLIC VARIABLES HERE WHEN NEEDED; DELETE THIS REGION IF NOT NEEDED

    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Threshold for display, if spin is below this value, DisableCanvas() will be called.
    /// </summary>
    [SerializeField]
    private float m_Threshold = 0.1f;

    /// <summary>
    /// Multiplied when calculating friction, scales friction effect.
    /// </summary>
    [SerializeField]
    private float m_Friction = 3;

    /// <summary>
    /// Scales speed  that is applied to turn the wheel after touch input.
    /// </summary>
    [SerializeField]
    private float m_Speed = 20;

    /// <summary>
    /// Index of controller to use for selection wheel (0/1).
    /// </summary>
    [SerializeField]
    private int m_ControllerIndex = 0;

    /// <summary>
    /// Specify where selected item should be positioned (clockwise, index in range of number of elem on circle).
    /// </summary>
    [SerializeField]
    private int m_SelectIndex = 0;

    /// <summary>
    /// Canvas with selection wheel, en- and disabled depending on wheel.
    /// </summary>
    [SerializeField]
    private Canvas m_Canvas;

    /// <summary>
    /// Rawimage representing background. if linked it will be rotated
    /// </summary>
    [SerializeField]
    private RawImage m_Background;

    /// <summary>
    /// This is used to calculate the spin after touch input stopped.
    /// </summary>
    private Vector2 m_PrevPos = Vector2.zero;

    /// <summary>
    /// Describes the spin, decreases over time with friction.
    /// </summary>
    private float m_CurSpin = 0;

    /// <summary>
    ///  Is selection wheel visible
    /// </summary>
    private bool m_IsVisible = false;

    /// <summary>
    /// before disabling wheel:was it moved again? Should it still be disabled?
    /// </summary>
    private bool m_Disabling = false;

    /// <summary>
    /// Radius of this object to rotate text on circle around centre.
    /// </summary>
    private float m_Radius;

    /// <summary>
    /// Overall angle of rotation around z axis, needed for selection.
    /// </summary>
    private float m_CurAngle = 180; //value chosen so that 0 degrees resemble 12o'clock / top part of the selection wheel

    /// <summary>
    /// number of text elems in selection wheel.
    /// </summary>
    private int m_ElemCount;

    /// <summary>
    /// currently selected text elem by index.
    /// </summary>
    private int m_SelectedTextIndex = -1;

    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Initialize Wheel: Place each child (text) elem evenly on wheel. 
    /// </summary>
    void Start()
    {
        Component[] children = GetComponentsInChildren(typeof(RectTransform));
        // assuming width and height identical 
        m_Radius = GetComponent<RectTransform>().rect.width;
        // ignore this element
        m_ElemCount = children.Length - 1;
        // stay within boundaries, not necessarily needed though
        if (m_SelectIndex > m_ElemCount) m_SelectIndex = m_SelectIndex % m_ElemCount;
        for (int i = 0; i < children.Length; i++)
        {
            RectTransform t = (RectTransform)children[i];
            if (t.gameObject != gameObject) //ignore this elem
            {
                // define point on outer circle called offset,start from 0 as 12 o'clock on the circle

                Vector2 offset = MathUtility.RotateVectorDegrees(360 / (m_ElemCount) * (i - 1));
                offset *= m_Radius; //apply length
                //move elem to that position on circle
                t.localPosition = new Vector3(offset.x, offset.y, 0);
            }
        }
        // counter turn elem to keep text straight
        if (m_Canvas)
        {
            // canvas.GetComponent<Canvas>().enabled = false;
            m_IsVisible = false;
        }
        HighlightSelection();
    }

    /// <summary>
    /// Update the wheel position and highlight the current selected item.
    /// </summary>
    void Update()
    {
        //SpinOnTouch(false);
        SpinWithInertia();
        HighlightSelection();
    }
    #endregion

    #region PUBLIC_METHODS

    // PUBLIC METHODS HERE WHEN NEEDED; DELETE THIS REGION IF NOT NEEDED

    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// Enables canvas containing wheel to be displayed
    /// </summary>
    private void EnableCanvas()
    {
        if (m_Canvas)
        {
            m_Canvas.GetComponent<Canvas>().enabled = true;
            m_IsVisible = true;
        }
    }
    /// <summary>
    /// Coroutine to wait shortly (0.5s) and disable canvas if user did not give new input meanwhile.
    /// </summary>
    /// <returns></returns>
    private IEnumerator DisableCanvasCoroutine()
    {
        //Debug.Log("disable gunction called...");
        yield return new WaitForSeconds(0.5f);

        if (m_Disabling) //if disable still desired (whilst waiting further user input might have changed that)
        {
            if (m_Canvas)
            {
                //Debug.Log("Disabling...");
                m_Canvas.GetComponent<Canvas>().enabled = false;
                m_IsVisible = false;
            }
        }
    }

    /// <summary>
    /// Detects the currently selected item fom the selection wheel using the current angle and highlights it.
    /// </summary>
    private void HighlightSelection()
    {
        int step = (360 / m_ElemCount);
        int tmp = m_ElemCount - (int)(m_CurAngle + step / 2) / step - 1;// map selection from 0 - (elems -1)
        tmp = MathUtility.Mod((tmp + m_SelectIndex - 1), m_ElemCount); // select the desired item clockwise, start at the top (-1)
        if (tmp == m_SelectedTextIndex)
        {
            return;
        }
        m_SelectedTextIndex = tmp;
        //inform of selection change
        VRUILogic.Instance.SetSelectedIndex(m_SelectedTextIndex);
        Text[] texts = GetComponentsInChildren<Text>();
        for (int i = 0; i < texts.Length; i++)
        {
            if (m_SelectedTextIndex != i)
            {
                texts[i].fontStyle = FontStyle.Normal;
                texts[i].color = Color.black;

            }
            else
            {
                texts[i].fontStyle = FontStyle.Bold;
                texts[i].color = Color.cyan;
            }
        }
    }

    /// <summary>
    /// Calculates the current angle based on the user input.
    /// The angle is set with respect to the previous angle and the distance the finger tracked. 
    /// </summary>
    /// <param name="spin">should the wheel spin after touch input</param>
    private void SpinOnTouch(bool spin)
    {

        bool touched = VRUILogic.Instance.GetTouchedInfo(m_ControllerIndex);
        Vector2 curPos = VRUILogic.Instance.GetTouchPosition(m_ControllerIndex);

        if (touched) // if input found
        {
            m_Disabling = false; // if change (input) occured whilst waiting for disable, do not disable
            if (!m_IsVisible) EnableCanvas();
            if (!m_PrevPos.Equals(Vector2.zero)) // angle does not change if new input just arrived
            {
                m_CurAngle += MathUtility.VectorToAngle(m_PrevPos) - MathUtility.VectorToAngle(curPos);
                m_CurAngle = MathUtility.WrapAngle(m_CurAngle);
            }
            m_PrevPos = curPos;

        }
        else
        {
            m_PrevPos = Vector2.zero;
            if (m_IsVisible && !m_Disabling && !spin) // prevent from calling multiple disable canvases if we're already disabling
            {
                StartCoroutine(DisableCanvasCoroutine());
                m_Disabling = true;
            }
        }
    }

    /// <summary>
    /// Updates the position of the spinning wheel based on the touch input
    /// </summary>
    private void SpinWithInertia()
    {
        // update the current spin with the mouse wheel
        Component[] children;
        Vector3 newPos;

        bool touched = VRUILogic.Instance.GetTouchedInfo(m_ControllerIndex);
        Vector2 curPos = VRUILogic.Instance.GetTouchPosition(m_ControllerIndex);

        // Visibility settings for spin effect
        if (!touched && m_CurSpin > -m_Threshold && m_CurSpin < m_Threshold) // if too slow and no input
        {
            m_CurSpin = 0;
            if (m_IsVisible && !m_Disabling) // prevent from calling multiple disable canvases if we're already disabling
            {
                StartCoroutine(DisableCanvasCoroutine());
                m_Disabling = true;
                return;
            }
        }

        if (touched)
        {
            if (!curPos.Equals(m_PrevPos))
            {
                // Debug.Log("Calculating spin");
                m_CurSpin = Vector2.Distance(curPos, m_PrevPos) * m_Speed;
                if (!MathUtility.TurningClockwise(m_PrevPos, curPos)) m_CurSpin *= -1;
            }
            SpinOnTouch(true);
        }
        else
        {
            m_PrevPos = Vector2.zero;
            m_CurSpin -= m_CurSpin * Time.deltaTime * m_Friction;
            // Debug.Log("applying spin and friction to angle");
            m_CurAngle = MathUtility.WrapAngle(m_CurAngle + (float)m_CurSpin);
        }
        // rotate children (each text)
        children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject)
            {
                float childangle = MathUtility.WrapAngle(360 - (m_CurAngle + (360 / m_ElemCount) * i));
                newPos = m_Radius * MathUtility.RotateVectorDegrees(childangle); //adjust position using new point on circle
                children[i].transform.localPosition = new Vector3(newPos.x, newPos.y, 0);
            }
        }
        //rotate background
        if (m_Background)
            m_Background.rectTransform.localRotation = Quaternion.Euler(0, 0, -m_CurAngle);
    }

    #endregion
}

