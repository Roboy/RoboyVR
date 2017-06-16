using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VRUILogic : Singleton<VRUILogic> {

    #region public database
    public int selectedMode = 0;
    public GameObject[] mode;

    #endregion

    #region private fields
    /// <summary>
    /// Array containing the current finger position on the touchpad if touched
    /// </summary>
    private Vector2[] touchData;

    /// <summary>
    /// Array containing information whether respective Touchpad on controller is currently being touched. 
    /// </summary>
    private bool[] touchedPad;
    #endregion

    /// <summary>
    /// Starts UI by disabling all except for specified modes.
    /// </summary>
    private void Start()
    {
        foreach(GameObject obj in Instance.mode)
        {
            obj.SetActive(false);
        }
        Instance.mode[selectedMode].gameObject.SetActive(true);

        touchData = new Vector2[2];
        touchData[0] = Vector2.zero;
        touchData[1] = Vector2.zero;
        touchedPad = new bool[2];
        touchedPad[0] = false;
        touchedPad[1] = false;
    }

    /// <summary>
    /// Function displays mode i and disables previous mode.
    /// </summary>
    /// <param name="i">index of mode</param>
    public void SelectedModeChanged(int i)
    {
        if (Instance.mode != null)
        {
            Debug.Log("New mode" + i);
            if (i < mode.Length && i >= 0)
            {
                mode[selectedMode].gameObject.SetActive(false);
                mode[i].gameObject.SetActive(true);
                selectedMode = i;
            }
        }
    }

    /// <summary>
    /// Method to update the given Touch data of the controller i.
    /// </summary>
    /// <param name="i">Index of the controller</param>
    /// <param name="newPos">Vector containing new position.</param>
    public void SetTouchData(int i, Vector2 newPos)
    {
        if(i< touchData.Length && i >= 0)
        {
            touchData[i] = newPos;
        }
    }

    /// <summary>
    /// Method to update, whether the Touchpad of the given Controller is being touched
    /// </summary>
    /// <param name="i"></param>
    /// <param name="touched"></param>
    public void SetTouched(int i, bool touched)
    {
        if (i < touchedPad.Length && i >= 0)
        {
            touchedPad[i] = touched;
        }
    }

    /// <summary>
    /// returns the boolean, if the touchpad of controller i is being touched.
    /// </summary>
    /// <param name="i">index of controller</param>
    /// <returns>touched yes/no</returns>
    public bool getTouchedInfo(int i)
    {
        if (i < touchedPad.Length && i >= 0)
        {
            return touchedPad[i];
        }
        return false;
    }

    public Vector2 getTouchPosition(int i)
    {
        if (i < touchData.Length && i >= 0)
        {
            return touchData[i];
        }
        return Vector2.zero;
    }
}
