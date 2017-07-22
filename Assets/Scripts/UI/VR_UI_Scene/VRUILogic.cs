using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Singleton Instance that functions as the main data base and core logic.
/// Variables can be set/requested using the methods. 
/// </summary>
public class VRUILogic : Singleton<VRUILogic>
{

    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// Current mode in enum type, helps different entities differ cases for display purposes
    /// </summary>
    public enum UIMode
    {
        Overview,
        Control,
        Cognition,
        Middleware
    };
    public interface ISubscriber
    {

        /// <summary>
        /// passes the information that the subscriber subscribed to.
        /// </summary>
        /// <param name="info"></param>
        void BeInformed(object info);

        /// <summary>
        /// informs the subscriber of updates. Passing no arguments only if objects deleted otherwise the halo will not be created
        /// </summary>
        void BeInformed();
    }
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
    /// skybox connected to camera
    /// </summary>
    [SerializeField]
    private Material skybox;
    /// <summary>
    /// For the main selection wheel, set which mode is default (offset)
    /// </summary>
    [SerializeField]
    private int m_selectIndex = 0;

    /// <summary>
    /// All modes from which can be chosen in this UI
    /// Names need to match the string representations in the UIMode Enumeration
    /// </summary>
    [SerializeField]
    private GameObject[] m_modes;

    #region notifications

    #region icons for notifications
    [Header("Icons for Notifications")]
    /// <summary>
    /// Texture of error icon
    /// </summary>
    [SerializeField]
    private Texture m_error;
    /// <summary>
    /// texture of warning icon
    /// </summary>
    [SerializeField]
    private Texture m_warning;
    /// <summary>
    /// texture of info icon
    /// </summary>
    [SerializeField]
    private Texture m_info;

    /// <summary>
    /// texture of debug icon
    /// </summary>
    [SerializeField]
    private Texture m_debug;
    #endregion
    /// <summary>
    /// Returns bool, if changes occured (set to true) since last check (set to false)
    /// </summary>
    public List<ISubscriber> m_NotificationSubscriber = new List<ISubscriber>();

    /// <summary>
    /// List containing all received error notifications
    /// </summary>
    private List<Notification> m_errorsList = new List<Notification>();

    /// <summary>
    /// List containing all received warnings
    /// </summary>
    private List<Notification> m_warningsList = new List<Notification>();

    /// <summary>
    /// List containing all received debug or similar additional information
    /// </summary>
    private List<Notification> m_debugsList = new List<Notification>();

    /// <summary>
    /// List containing all information sent as notifications
    /// </summary>
    private List<Notification> m_infosList = new List<Notification>();
    #endregion
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS1
    /// <summary>
    /// Starts UI by disabling all except for specified modes.
    /// </summary>
    private void Awake()
    {
        if (m_modes != null && m_modes.Length > 0)
        {
            foreach (GameObject obj in m_modes)
            {
                obj.SetActive(false);
            }
            m_modes[m_selectedMode].gameObject.SetActive(true);
        }

        m_touchData = new Vector2[2];
        m_touchData[0] = Vector2.zero;
        m_touchData[1] = Vector2.zero;
        m_touchedPad = new bool[2];
        m_touchedPad[0] = false;
        m_touchedPad[1] = false;
        Skybox box = m_headset.gameObject.AddComponent<Skybox>();
        box.material = skybox;
    }
    #endregion

    #region PUBLIC_METHODS
    #region getters

    public Texture GetIconTexture(DummyStates.MessageType iconType)
    {
        switch (iconType)
        {
            case DummyStates.MessageType.INFO:
                return m_info;
            case DummyStates.MessageType.DEBUG:
                return m_debug;
            case DummyStates.MessageType.WARNING:
                return m_warning;
            case DummyStates.MessageType.ERROR:
                return m_error;
            default:
                return null;
        }
    }

    /// <summary>
    /// Returns EnumState representation of currently selected state.
    /// </summary>
    /// <returns></returns>
    public UIMode GetCurrentMode()
    {
        //TODO: really doggy..... hopefully not higher index than uimode-entities
        return (UIMode)m_selectedMode;
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
    #endregion

    #region setters

    /// <summary>
    /// Method to update the given Touch data of the controller i.
    /// </summary>
    /// <param name="i">Index of the controller</param>
    /// <param name="newPos">Vector containing new position.</param>
    public void SetTouchPosition(int i, Vector2 newPos)
    {
        if (i < m_touchData.Length && i >= 0)
        {
            m_touchData[i] = newPos;
        }
    }

    /// <summary>
    /// Method to update, whether the Touchpad of the given Controller is being touched
    /// </summary>
    /// <param name="i"></param>
    /// <param name="touched"></param>
    public void SetTouchedInfo(int i, bool touched)
    {
        if (i < m_touchedPad.Length && i >= 0)
        {
            m_touchedPad[i] = touched;
        }
    }
    #endregion

    #region others
    /// <summary>
    /// Function displays mode with index i and disables previous mode.
    /// </summary>
    /// <param name="i">index of mode</param>
    public void SelectedModeChanged(int i)
    {
        if (Instance.m_modes != null)
        {
            i += m_selectIndex;
            i %= m_modes.Length;
            Debug.Log("New mode" + i);
            if (i < m_modes.Length && i >= 0)
            {
                m_modes[m_selectedMode].gameObject.SetActive(false);
                m_modes[i].gameObject.SetActive(true);
                m_selectedMode = i;
            }
        }
    }
    #endregion

    #region notifications 
    /// <summary>
    /// The specified notification note is added to the respective list of existing notifications (warning, debug or error as of now)
    /// </summary>
    /// <param name="note">Notification which is to be added</param>
    public void AddNotification(Notification note)
    {
        if (note != null)
        {
            Debug.Log("New notification in VRUILogic");
            switch (note.GetNotificationType())
            {
                case DummyStates.MessageType.DEBUG:
                    m_debugsList.Add(note);
                    break;
                case DummyStates.MessageType.WARNING:
                    m_warningsList.Add(note);
                    break;
                case DummyStates.MessageType.ERROR:
                    m_errorsList.Add(note);
                    break;
                case DummyStates.MessageType.INFO:
                    m_infosList.Add(note);
                    break;
                default:
                    Debug.Log("[VRUILogic]This notification type is not implemented yet!" + note.GetNotificationType().ToString());
                    break;
            }
            InformNotificationSubscribers(note);
        }
    }

    /// <summary>
    /// Initialises new notification with given values, creates gameobject and adds notification component. 
    /// Notification is inserted in database. 
    /// </summary>
    /// <param name="messageType">message type of notification</param>
    /// <param name="state">state of notification</param>
    /// <param name="objectName">name of Roboy body part notification is related to (can be null)</param>
    /// <param name="timeFrame">time frame in which notification is valid</param>
    /// <returns></returns>
    public Notification AddNewNotification(DummyStates.MessageType messageType, DummyStates.State state, string objectName , float timeFrame)
    {
        GameObject obj = new GameObject();
        obj.name = "Notification"; //unity automatically changes name if multiple instances occure -> e.g. "Notification(3)" 
        Notification note = obj.AddComponent<Notification>();
        note.SetupNotification(messageType, state, objectName, timeFrame);
        AddNotification(note);
        return note;
    }

    /// <summary>
    /// [Only Called By Notifications!!!]
    /// To delete notifications and halo / references/ effects, call notification.DeleteNotification() instead!!!!
    /// Removes notification from internal database and informs subscribers of change. 
    /// </summary>
    /// <param name="note"></param>
    public void RemoveNotification(Notification note)
    {
        if (!note) return;
        switch (note.GetNotificationType())
        {
            case DummyStates.MessageType.INFO:
                m_infosList.Remove(note);
                break;
            case DummyStates.MessageType.DEBUG:
                m_debugsList.Remove(note);
                break;
            case DummyStates.MessageType.WARNING:
                m_warningsList.Remove(note);
                break;
            case DummyStates.MessageType.ERROR:
                m_errorsList.Remove(note);
                break;
            default:
                break;
        }
        InformNotificationSubscribers(null);
    }

    /// <summary>
    /// Deletes all currently stored notifications and removes links to roboy
    /// </summary>
    public void ClearAllNotifications()
    {
        foreach (Notification note in m_errorsList)
        {
            note.DeleteNotification();
        }
        foreach (Notification note in m_warningsList)
        {
            note.DeleteNotification();
        }
        foreach (Notification note in m_debugsList)
        {
            note.DeleteNotification();
        }
        m_errorsList = new List<Notification>();
        m_warningsList = new List<Notification>();
        m_debugsList = new List<Notification>();
        InformNotificationSubscribers(null);
    }
    /// <summary>
    /// Returns list of all currently saved errors
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllErrors()
    {
        return m_errorsList;
    }

    /// <summary>
    /// returns list of all currently saved warnings
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllWarnings()
    {
        return m_warningsList;
    }

    /// <summary>
    /// returns list of all currently saved debug messages
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllDebugs()
    {
        return m_debugsList;
    }

    /// <summary>
    /// returns list of all currently saved info messages
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllInfos()
    {
        return m_infosList;
    }
    /// <summary>
    /// Adds subscriber to list of classes to be informed of new notifications
    /// </summary>
    /// <param name="subscriber">ISubscriber which is to be subscribed</param>
    public void SubscribeNotifications(ISubscriber subscriber)
    {
        if (subscriber != null && !m_NotificationSubscriber.Contains(subscriber))
        {
            m_NotificationSubscriber.Add(subscriber);
        }
    }
    #endregion
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// for each subscribed interface, inform method with new notification called
    /// </summary>
    /// <param name="note">Notification of which to inform subscribers, if null -> object removed</param>
    private void InformNotificationSubscribers(Notification note)
    {
        foreach (ISubscriber subscriber in m_NotificationSubscriber)
        {
            if (note == null)
            {
                subscriber.BeInformed();
            }
            else
                subscriber.BeInformed(note);
        }
    }
    #endregion
}
