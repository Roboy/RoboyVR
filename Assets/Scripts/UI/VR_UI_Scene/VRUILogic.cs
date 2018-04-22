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
        Cognition,
        Control,
        Middleware
    };

    /// <summary>
    /// Subscriber interface which is to be implemented by instances that need/want to be informed of changes
    /// </summary>
    public interface ISubscriber
    {

        /// <summary>
        /// passes the information that the subscriber subscribed to.
        /// </summary>
        /// <param name="info"></param>
        void BeInformed(object info);

        /// <summary>
        /// informs the subscriber of updates. Passing no arguments only if objects deleted / no new ones added but changes applied
        /// </summary>
        void BeInformed();
    }
    #endregion

    #region PRIVATE_MEMBER_VARIABLES    
    #region general
    /// <summary>
    /// skybox connected to camera
    /// </summary>
    [SerializeField]
    private Material m_Skybox;

    /// <summary>
    /// All modes from which can be chosen in this UI
    /// Names need to match the string representations in the UIMode Enumeration
    /// </summary>
    [SerializeField]
    private GameObject[] m_Modes;

    /// <summary>
    /// Reference to Roboy for position references
    /// </summary>
    private GameObject m_Roboy;

    /// <summary>
    /// Gameobject which contains all UI components
    /// </summary>
    [SerializeField]
    private GameObject m_UIComponents;
    #endregion

    #region userInpur Related
    /// <summary>
    /// Contains the mode of the currently selected tool the user works with
    /// </summary>
    private ModeManager.GUIMode m_GUIMode;

    /// <summary>
    /// Specifies whether user is currently changing his tool (hand, time tool, pointer ....) or not
    /// </summary>
    private bool m_UserIsSelectingTool;

    /// <summary>
    /// Specifies whether user is currently changing his mode (BeRoboy, Spawn mode, Gui mode....) or not
    /// </summary>
    private bool m_UserIsSelectingMode;
    /// <summary>
    /// Array containing information whether respective Touchpad on controller is currently being touched. 
    /// </summary>
    private bool[] m_TouchedPad;

    /// <summary>
    /// Array containing the current finger position on the touchpad if touched
    /// </summary>
    private Vector2[] m_TouchData;

    /// <summary>
    /// VR Headset camera, for position and rotation information
    /// </summary>
    [SerializeField]
    private Camera m_Headset;

    /// <summary>
    /// This value specifies the currently selected mode
    /// </summary>
    private int m_SelectedMode = 0;

    /// <summary>
    /// For the main selection wheel, set which mode is default (offset)
    /// </summary>
    [SerializeField]
    private int m_SelectIndex = 0;
    #endregion

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
    private List<Notification> m_ErrorsList = new List<Notification>();

    /// <summary>
    /// List containing all received warnings
    /// </summary>
    private List<Notification> m_WarningsList = new List<Notification>();

    /// <summary>
    /// List containing all received debug or similar additional information
    /// </summary>
    private List<Notification> m_DebugsList = new List<Notification>();

    /// <summary>
    /// List containing all information sent as notifications
    /// </summary>
    private List<Notification> m_InfosList = new List<Notification>();

    /// <summary>
    /// Container to set as parent of notifications
    /// </summary>
    private GameObject m_NotificationsContainer;
    #endregion

    #region tendons
    /// <summary>
    /// parent gameObject to all tendons
    /// </summary>
    private GameObject m_TendonContainer;

    /// <summary>
    /// This container holds references to all wirepoints which could not be matched to their respective Roboy part.
    /// </summary>
    private GameObject m_WirepointContainer;
    /// <summary>
    /// List containing all registered tendons 
    /// </summary>
    private List<Tendon> m_Tendons = new List<Tendon>();
    #endregion
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Starts UI by disabling all except for specified modes. Sets / Initializes further values
    /// </summary>
    private void Awake()
    {
        //general
        if (m_Modes != null && m_Modes.Length > 0)
        {
            foreach (GameObject obj in m_Modes)
            {
                obj.SetActive(false);
            }
            m_Modes[m_SelectIndex % m_Modes.Length].gameObject.SetActive(true);
        }
        m_SelectedMode = m_SelectIndex;

        m_TouchData = new Vector2[2];
        m_TouchData[0] = Vector2.zero;
        m_TouchData[1] = Vector2.zero;
        m_TouchedPad = new bool[2];
        m_TouchedPad[0] = false;
        m_TouchedPad[1] = false;
        if (m_Skybox)
        {
            Skybox previosBox = m_Headset.gameObject.GetComponent<Skybox>();
            if (previosBox)
            {
                previosBox.material = m_Skybox;
            } else {
                Skybox box = m_Headset.gameObject.AddComponent<Skybox>();
                box.material = m_Skybox;
            }
        }
        //roboy reference
        m_Roboy = RoboyManager.Instance.Roboy.gameObject;

        //notifications
        m_NotificationsContainer = new GameObject("NotificationContainer");
        if (m_UIComponents)
            m_NotificationsContainer.transform.SetParent(m_UIComponents.transform);
        //tendons
        m_TendonContainer = new GameObject("TendonContainer");
        if (m_Modes != null && m_Modes.Length > ((int)UIMode.Middleware))
        {
            Debug.Log("[VRUILogic] Tendoncontainer set as child obj");
            //TODO: for now: always have tendons visible
            //m_TendonContainer.transform.SetParent(m_Modes[(int)UIMode.Middleware].transform);
        }
        //Wirepoints of tendons
        m_WirepointContainer = new GameObject("WirepointDefaultContainer");
        //TODO: for now: always have tendons visible
        //if (m_UIComponents)
        //m_WirepointContainer.transform.SetParent(m_UIComponents.transform);
    }
    #endregion

    #region PUBLIC_METHODS

    #region Other

    /// <summary>
    /// Sets, whether the user currently changes his tool (time/selector/hand ...) or if he is not changing it
    /// Hides UI in order to ease selection process
    /// </summary>
    /// <param name="state"></param>
    public void SetToolWheelState(bool state)
    {
        m_UserIsSelectingTool = state;
        if (state) //if currently selecting, disable UI to have less obstacles and objects while selecting
            SetUserInterfaceComponentsState(false);
        else if (m_GUIMode == ModeManager.GUIMode.GUIViewer && !m_UserIsSelectingMode)
        {
            SetUserInterfaceComponentsState(true);
        }
    }
    /// <summary>
    /// Sets the internal GUIMode state according to given mode in order to decide whether to display UI or hide it.
    /// Hides UI in case the mode is not GUIViewer
    /// </summary>
    /// <param name="currentMode">newly selected GUI mode</param>
    public void SetGUIMode(ModeManager.GUIMode currentMode)
    {
        Debug.Log("[VRUILogic] Setting GUI mode " + currentMode.ToString());
        m_GUIMode = currentMode;
        if (m_GUIMode != ModeManager.GUIMode.GUIViewer) // if not sele
            SetUserInterfaceComponentsState(false);
        else if (!m_UserIsSelectingTool && !m_UserIsSelectingMode)
        {
            SetUserInterfaceComponentsState(true);
        }

    }

    /// <summary>
    /// sets whether GUI mode selection currently displayed (-> disable UI components to ease selection process)
    /// </summary>
    /// <param name="state"></param>
    public void SetGUIModeSelecting(bool state)
    {
        m_UserIsSelectingMode = state;
        if (state) //disable UI if selecting
            SetUserInterfaceComponentsState(false);
        //if not selecting anything and in GUIViewer mode
        else if(!m_UserIsSelectingTool && m_GUIMode == ModeManager.GUIMode.GUIViewer)
        {
            SetUserInterfaceComponentsState(true);
        }
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
    /// Activates or deactivates all UI components according to specified boolean
    /// </summary>
    /// <param name="active">active: = true-> enable components</param>
    public void SetUserInterfaceComponentsState(bool active)
    {
        if (m_UIComponents)
            m_UIComponents.SetActive(active);
    }
    #endregion

    #region UserInput-related
    /// <summary>
    /// Returns EnumState representation of currently selected state.
    /// </summary>
    /// <returns></returns>
    public UIMode GetCurrentMode()
    {
        //TODO: really doggy..... hopefully not higher index than uimode-entities
        return (UIMode)m_SelectedMode;
    }

    /// <summary>
    /// returns the boolean, if the touchpad of controller i is being touched.
    /// </summary>
    /// <param name="i">index of controller</param>
    /// <returns>touched yes/no</returns>
    public bool GetTouchedInfo(int i)
    {
        if (i < m_TouchedPad.Length && i >= 0)
        {
            return m_TouchedPad[i];
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
        if (i < m_TouchData.Length && i >= 0)
        {
            return m_TouchData[i];
        }
        return Vector2.zero;
    }

    /// <summary>
    /// Method to update the given Touch data of the controller i.
    /// </summary>
    /// <param name="i">Index of the controller</param>
    /// <param name="newPos">Vector containing new position.</param>
    public void SetTouchPosition(int i, Vector2 newPos)
    {
        if (i < m_TouchData.Length && i >= 0)
        {
            m_TouchData[i] = newPos;
        }
    }

    /// <summary>
    /// Method to update, whether the Touchpad of the given Controller is being touched
    /// </summary>
    /// <param name="i"></param>
    /// <param name="touched"></param>
    public void SetTouchedInfo(int i, bool touched)
    {
        if (i < m_TouchedPad.Length && i >= 0)
        {
            m_TouchedPad[i] = touched;
        }
    }

    /// <summary>
    /// returns current rotation of headset (camera rig)
    /// </summary>
    /// <returns>Current rotation in quaternion</returns>
    public Quaternion GetCameraRotation()
    {
        return m_Headset.transform.rotation;
    }

    /// <summary>
    /// returns the headset (camera of the scene)
    /// </summary>
    /// <returns>main camera</returns>
    public Camera GetCamera()
    {
        return m_Headset;
    }

    /// <summary>
    /// returns reference to the main roboy of the scene (which existed from the beginning)
    /// </summary>
    /// <returns></returns>
    public GameObject GetRoboy()
    {
        return m_Roboy;
    }
    /// <summary>
    /// Function displays mode with index i and disables previous mode.
    /// </summary>
    /// <param name="i">index of mode</param>
    public void SetSelectedIndex(int i)
    {
        if (m_Modes != null)
        {
            i += m_SelectIndex;
            i %= m_Modes.Length;
            //Debug.Log("[VRUILogic] New mode: " + i);
            if (i < m_Modes.Length && i >= 0)
            {
                m_Modes[m_SelectedMode].gameObject.SetActive(false);
                m_Modes[i].gameObject.SetActive(true);
                m_SelectedMode = i;
            }
        }
    }
    #endregion

    #region notifications 
    /// <summary>
    /// Returns respective texture for passed message type
    /// </summary>
    /// <param name="iconType">message type for which texture is requested</param>
    /// <returns></returns>
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
    /// The specified notification note is added to the respective list of existing notifications (warning, debug or error as of now).
    /// It is added to the NotificationContainer (as child obj)
    /// </summary>
    /// <param name="note">Notification which is to be added</param>
    public void AddNotification(Notification note)
    {
        if (note != null)
        {
            note.transform.parent = m_NotificationsContainer.transform;
            //Debug.Log("New notification in VRUILogic");
            switch (note.GetNotificationType())
            {
                case DummyStates.MessageType.DEBUG:
                    m_DebugsList.Add(note);
                    break;
                case DummyStates.MessageType.WARNING:
                    m_WarningsList.Add(note);
                    break;
                case DummyStates.MessageType.ERROR:
                    m_ErrorsList.Add(note);
                    break;
                case DummyStates.MessageType.INFO:
                    m_InfosList.Add(note);
                    break;
                default:
                    Debug.Log("[VRUILogic] This notification type is not implemented yet!" + note.GetNotificationType().ToString());
                    break;
            }
            //Debug.Log("Informing subscribers");
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
    public Notification AddNewNotification(DummyStates.MessageType messageType, DummyStates.State state, string objectName, float timeFrame)
    {
        GameObject obj = new GameObject();
        obj.name = "Notification"; //unity automatically changes name if multiple instances occure -> e.g. "Notification(3)" 
        Notification note = obj.AddComponent<Notification>();
        note.Initialize(messageType, state, objectName, timeFrame);
        AddNotification(note);
        note.transform.SetParent(m_NotificationsContainer.transform);
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
                m_InfosList.Remove(note);
                break;
            case DummyStates.MessageType.DEBUG:
                m_DebugsList.Remove(note);
                break;
            case DummyStates.MessageType.WARNING:
                m_WarningsList.Remove(note);
                break;
            case DummyStates.MessageType.ERROR:
                m_ErrorsList.Remove(note);
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
        foreach (Notification note in m_ErrorsList)
        {
            note.DeleteNotification();
        }
        foreach (Notification note in m_WarningsList)
        {
            note.DeleteNotification();
        }
        foreach (Notification note in m_DebugsList)
        {
            note.DeleteNotification();
        }
        m_ErrorsList = new List<Notification>();
        m_WarningsList = new List<Notification>();
        m_DebugsList = new List<Notification>();
        InformNotificationSubscribers(null);
    }

    /// <summary>
    /// Returns list of all currently saved errors
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllErrors()
    {
        return m_ErrorsList;
    }

    /// <summary>
    /// returns list of all currently saved warnings
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllWarnings()
    {
        return m_WarningsList;
    }

    /// <summary>
    /// returns list of all currently saved debug messages
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllDebugs()
    {
        return m_DebugsList;
    }

    /// <summary>
    /// returns list of all currently saved info messages
    /// </summary>
    /// <returns></returns>
    public List<Notification> GetAllInfos()
    {
        return m_InfosList;
    }

    /// <summary>
    /// Adds subscriber to list of classes to be informed of new notifications.
    /// If already added, nothing will happen.
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

    #region tendons

    /// <summary>
    /// Adds new tendon to list of existing tendons. Sets all references and sets all references
    /// If tendon with this id already exists, nothing happens.
    /// </summary>
    /// <param name="tendonID">ID of new tendon</param>
    /// <param name="positions">position of wirepoints in world space</param>
    /// <param name="objectNames">names of roboy parts which are linked to each point</param>
    /// <param name="maxforce"></param>
    public void AddTendon(int tendonID, Vector3[] positions, string[] objectNames, float maxforce)
    {
        GameObject tendonObj = new GameObject();
        tendonObj.name = "Tendon " + tendonID;
        tendonObj.transform.SetParent(m_TendonContainer.transform);
        Tendon tendon = tendonObj.AddComponent<Tendon>();
        for (int i = 0; i < positions.Length; i++)
        {//assume Roboy in simulation around (0,0,0) since this roboy here is transformed to fit in the cave -> adapt
            positions[i] += m_Roboy.transform.position;
        }
        tendon.Initialize(tendonID, positions, objectNames, maxforce);
        AddTendon(tendon);
    }

    /// <summary>
    /// In case wirepoints could not be linked to the respective Roboy parts, the points are stored in here.
    /// </summary>
    /// <returns></returns>
    public GameObject GetDefaultWirepointContainer()
    {
        return m_WirepointContainer;
    }
    /// <summary>
    /// updates tendon with new force value in case tendon is registered/exists
    /// </summary>
    /// <param name="tendonID">tendon which is to be updated</param>
    /// <param name="newForce">updated value</param>
    public void UpdateTendon(int tendonID, float newForce)
    {
        if (m_Tendons.Count - 1 >= tendonID && tendonID >= 0)
        {
            if (m_Tendons[tendonID])
                m_Tendons[tendonID].UpdateTendonForce(newForce);
        }
    }

    /// <summary>
    /// Adds the specified tendon offset to the current position
    /// </summary>
    /// <param name="tendonID"></param>
    /// <param name="newOffset"></param>
    public void ApplyTendonOffset(int tendonID, Vector3 offset)
    {
        if (m_Tendons.Count - 1 >= tendonID && tendonID >= 0)
        {
            if (m_Tendons[tendonID])
                m_Tendons[tendonID].ApplyOffset(offset);
        }
    }

    /// <summary>
    /// Returns tendon with respective ID. If not existant, null returned
    /// </summary>
    /// <param name="tendonID">ID of tendon in question</param>
    /// <returns></returns>
    public Tendon GetTendon(int tendonID)
    {
        if (m_Tendons.Count - 1 >= tendonID && tendonID >= 0)
        {
            return m_Tendons[tendonID];
        }
        return null;
    }

    /// <summary>
    /// Returns gameobject with specified name which is child obj of Roboy. null if not found
    /// </summary>
    /// <param name="bodypartname">name of the part in question</param>
    /// <returns></returns>
    public GameObject GetBodyPart(string bodypartname)
    {
        if (m_Roboy)
        {
            Transform temp = m_Roboy.transform.Find(bodypartname);
            if (temp) return temp.gameObject;
        }
        return null;
    }
    #endregion

    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// Adds tendon to list of existing tendons in case no tendon with this id
    /// exists so far and it's not in the list already.
    /// </summary>
    /// <param name="tendon"></param>
    private void AddTendon(Tendon tendon)
    {
        //Debug.Log("Adding tendon (tendon)");
        if (tendon && !m_Tendons.Contains(tendon)) //if not inserted already
        {
            int index = tendon.GetTendonID();
            //add tendon to appropriate spot: 
            if (index >= m_Tendons.Count || !m_Tendons[index]) //if no tendon with same id existing yet
            {
                while (m_Tendons.Count < index) //fill list until tendon can be inserted at index 
                {
                    m_Tendons.Insert(m_Tendons.Count, null);
                }
                m_Tendons.Insert(index, tendon);

            }
        }
        else
        {
            Destroy(tendon.gameObject); //destroy gameobject and script linked to it
        }
    }

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
