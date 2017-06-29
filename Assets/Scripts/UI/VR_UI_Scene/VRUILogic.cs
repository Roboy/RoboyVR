using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Singleton Instance that functions as the main data base and core logic.
/// Variables can be set/requested using the methods. 
/// </summary>
public class VRUILogic : Singleton<VRUILogic> {

    #region PUBLIC_MEMBER_VARIABLES
    public GameObject[] mode;

    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Array containing the current finger position on the touchpad if touched
    /// </summary>
    private Vector2[] touchData;
  
    /// <summary>
    /// This value specifies the currently selected mode
    /// </summary>
    private int selectedMode = 0;
  
    /// <summary>
    /// Array containing information whether respective Touchpad on controller is currently being touched. 
    /// </summary>
    private bool[] touchedPad;

    /// <summary>
    /// VR Headset camera, for position and rotation information
    /// </summary>
    [SerializeField]
    private Camera headset;

    /// <summary>
    /// For the main selection wheel, set which mode is default (offset)
    /// </summary>
    [SerializeField]
    private int selectIndex = 0;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS1
    /// <summary>
    /// Starts UI by disabling all except for specified modes.
    /// </summary>
    private void Awake()
    {
        if (mode != null && mode.Length > 0)
        {
            foreach (GameObject obj in mode)
            {
                obj.SetActive(false);
            }
            mode[selectedMode].gameObject.SetActive(true);
        }
        
        touchData = new Vector2[2];
        touchData[0] = Vector2.zero;
        touchData[1] = Vector2.zero;
        touchedPad = new bool[2];
        touchedPad[0] = false;
        touchedPad[1] = false;

        
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Function displays mode i and disables previous mode.
    /// </summary>
    /// <param name="i">index of mode</param>
    public void SelectedModeChanged(int i)
    {
        if (Instance.mode != null)
        {
            i += selectIndex;
            i %= mode.Length;
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
    public void SetTouchPosition(int i, Vector2 newPos)
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
    public bool GetTouchedInfo(int i)
    {
        if (i < touchedPad.Length && i >= 0)
        {
            return touchedPad[i];
        }
        return false;
    }
    /// <summary>
    /// returns a vector containing the position on the touchpad with index i
    /// </summary>
    /// <param name="i">touchpad index</param>
    /// <returns>Current position on touchpad with max length == 1</returns>
    public Vector2 GetTouchPosition(int i)
    {
        if (i < touchData.Length && i >= 0)
        {
            return touchData[i];
        }
        return Vector2.zero;
    }
    /// <summary>
    ///  Returns a list of selected objects from the selectorManager containing the info.
    /// </summary>
    /// <returns>List of objects</returns>
    public List<SelectableObject> GetSelectedParts()
    {
        return SelectorManager.Instance.SelectedParts;
    }
    #endregion

    /// <summary>
    /// returns current rotation of headset (camera rig)
    /// </summary>
    /// <returns>Current rotation in quaternion</returns>
    public Quaternion GetCameraRotation()
    {
        return headset.transform.rotation;
    }
}
