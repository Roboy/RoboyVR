using UnityEngine;
//TODO: include enumeration file
/// <summary>
/// This class provides structures and functionalities for Notifications.
/// Creating these depending on the message type as well as provision of content depending on the type
/// </summary>
public class Notification
{

    /// <summary>
    /// Basic Notification constructor, creating note with specified type and state
    /// </summary>
    /// <param name="type">What type of message (warning, error ...)</param>
    /// <param name="state">What state is the notification depicting</param>
    /// <param name="objID">The ID of the concerned body part</param>
    public Notification(DummyStates.MessageType type, DummyStates.State state, int objID)
    {
        m_state = state;
        m_type = type;
        m_content = null;
        m_objectid = objID;
        GetConcernedRoboyPart().GetComponent<RoboyPart>().AddNotification(this);
    }

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Time of creation of notification
    /// </summary>
    private float timestamp;

    /// <summary>
    /// The concerned object for which this notification was sent
    /// </summary>
    private int m_objectid;

    /// <summary>
    /// The reason/issue why this message was created. 
    /// </summary>
    private DummyStates.State m_state;

    /// <summary>
    /// What type of notification: debug, warning, error ....
    /// </summary>
    private DummyStates.MessageType m_type;

    /// <summary>
    /// Bodypart of Roboy that is concerned, specified with ID as well.
    /// </summary>
    private GameObject m_bodyPart;

    /// <summary>
    /// Content of  Notification.
    /// Depending on message type, different content is provided
    /// </summary>
    private Content m_content;

    /// <summary>
    /// class defining the content of a message
    /// </summary>
    private class Content
    {
        /// <summary>
        /// provides Information in form of a string.
        /// </summary>
        public string m_information;
        /// <summary>
        /// 
        /// </summary>
        public string m_Origin;

        public string GetContent()
        {
            if (m_information.Equals(string.Empty))
            {
                if (m_Origin.Equals(string.Empty))
                {
                    return string.Empty;
                }
                else
                {
                    return "At: " + m_Origin;
                }
            }
            else
            {
                if (m_Origin.Equals(string.Empty))
                {
                    return m_information;
                }
                else
                {
                    return m_information + ", at: " + m_Origin;
                }
            }
        }

    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// returns additional info if available, else returns empty string
    /// </summary>
    /// <returns>additonal info, everything in one line</returns>
    public string GetAdditionalInfo()
    {
        if (m_content != null)
        {
            return m_content.GetContent();
        }
        else
        {
            return string.Empty;
        }
    }

    /// <summary>
    /// returns the type of the notification
    /// </summary>
    /// <returns>DummyState.MessageType</returns>
    public DummyStates.MessageType GetNotificationType()
    {
        return m_type;
    }

    /// <summary>
    /// returns information about note in the following format:  [type]: [state]
    /// </summary>
    /// <returns> [type]: [state]</returns>
    public string GetBasicInfo()
    {
        return m_type.ToString() + ": " + m_state.ToString();
    }

    /// <summary>
    /// Displays Halo around concerned GameObject. Colour is chosen according to notification type
    /// </summary>
    public void DisplayHalo()
    {
        if (m_type != DummyStates.MessageType.DEBUG)
            GetConcernedRoboyPart().GetComponent<RoboyPart>().UpdateHalo();

    }

    /// <summary>
    /// disables halo if this notification is the only note causing a halo. 
    /// Updates halo for other notifications if present.
    /// [NOT IMPLEMENTED YET]
    /// </summary>
    public void DisableHalo()
    {
        //TODO: Disable only one notification: remove from list? 
        //GetConcernedRoboyPart().GetComponent<RoboyPart>().Disable();
    }
    /// <summary>
    /// returns gameobject which is concerned for this notification
    /// </summary>
    /// <returns></returns>
    public GameObject GetConcernedRoboyPart()
    {
        if (!m_bodyPart)
        {
            //TODO find roboy body part according to obj id
            Debug.Log("looking for dummy game object: oberarm_right");
            m_bodyPart = GameObject.Find("oberarm_right");
        }
        return m_bodyPart;
    }

    /// <summary>
    /// automatically removes this message link to roboy, still in game list
    /// </summary>
    public void UnlinkFromRoboy()
    {
        GetConcernedRoboyPart().GetComponent<RoboyPart>().RemoveNotification(this);

        GetConcernedRoboyPart().GetComponent<RoboyPart>().DisableErrors();
    }
    /*
    /// <summary>
    /// 
    /// </summary>
    public void AddContent(string info)
    {
        if(m_type != DummyStates.MessageType.ERROR)
        {
            Debug.Log("Only supporting additional Notification content for error messages. Aborting");
            return;
        }
    }*/
    #endregion

    #region PRIVATE_METHODS
    #endregion
}
