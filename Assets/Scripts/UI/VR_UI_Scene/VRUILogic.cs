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
    private Vector2[] m_touchData;
  
    /// <summary>
    /// This value specifies the currently selected mode
    /// </summary>
    private int m_selectedMode = 0;
  
    /// <summary>
    /// Array containing information whether respective Touchpad on controller is currently being touched. 
    /// </summary>
    private bool[] m_touchedPad;

    /// <summary>
    /// VR Headset camera, for position and rotation information
    /// </summary>
    [SerializeField]
    private Camera m_headset;

    /// <summary>
    /// For the main selection wheel, set which mode is default (offset)
    /// </summary>
    [SerializeField]
    private int m_selectIndex = 0;
    #endregion

    private List<Notification> m_notifications;

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
            mode[m_selectedMode].gameObject.SetActive(true);
        }
        
        m_touchData = new Vector2[2];
        m_touchData[0] = Vector2.zero;
        m_touchData[1] = Vector2.zero;
        m_touchedPad = new bool[2];
        m_touchedPad[0] = false;
        m_touchedPad[1] = false;

        m_notifications = new List<Notification>();
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
            i += m_selectIndex;
            i %= mode.Length;
            Debug.Log("New mode" + i);
            if (i < mode.Length && i >= 0)
            {
                mode[m_selectedMode].gameObject.SetActive(false);
                mode[i].gameObject.SetActive(true);
                m_selectedMode = i;
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
        if(i< m_touchData.Length && i >= 0)
        {
            m_touchData[i] = newPos;
        }
    }

    /// <summary>
    /// Method to update, whether the Touchpad of the given Controller is being touched
    /// </summary>
    /// <param name="i"></param>
    /// <param name="touched"></param>
    public void SetTouched(int i, bool touched)
    {
        if (i < m_touchedPad.Length && i >= 0)
        {
            m_touchedPad[i] = touched;
        }
    }

    /// <summary>
    /// returns the boolean, if the touchpad of controller i is being touched.
    /// </summary>
    /// <param name="i">index of controller</param>
    /// <returns>touched yes/no</returns>
    public bool GetTouchedInfo(int i)
    {
        if (i < m_touchedPad.Length && i >= 0)
        {
            return m_touchedPad[i];
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
        if (i < m_touchData.Length && i >= 0)
        {
            return m_touchData[i];
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
    
    /// <summary>
    /// returns current rotation of headset (camera rig)
    /// </summary>
    /// <returns>Current rotation in quaternion</returns>
    public Quaternion GetCameraRotation()
    {
        return m_headset.transform.rotation;
    }

    /// <summary>
    /// The specified notification note is added to the list of existing notifications
    /// </summary>
    /// <param name="note">Notification which is to be added</param>
    public void AddNotification(Notification note)
    {
        if(note != null)
        {
            m_notifications.Add(note);
        }
    }

    /// <summary>
    /// returns list of all errors currently saved
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllErrors()
    {
        List<Notification> errors = new List<Notification>();
        foreach( Notification note in m_notifications)
        {
            if( note.getType() == DummyStates.MessageType.ERROR)
            {
                errors.Add(note);
            }
        }
        return errors;
    }
    #endregion
}
