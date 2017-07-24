using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//TODO: include enumeration file
/// <summary>
/// This class provides structures and functionalities for Notifications.
/// Creating these depending on the message type as well as provision of content depending on the type.
/// Monobehaviour needed to call Destroy() and Invoke() functions
/// </summary>
public class Notification : MonoBehaviour
{
    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Time of creation of notification
    /// </summary>
    private float timestamp;

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
    /// Time that this element was created (not in unity but on the simulation/real roboy side)
    /// </summary>
    private DateTime m_timestamp;

    /// <summary>
    /// Time frame length (in seconds), how long message is valid and will exist. Automatic deletion after end
    /// </summary>
    private double m_timeFrame;
    /// <summary>
    /// Content of  Notification.
    /// Depending on message type, different content is provided
    /// </summary>
    private Content m_content;

    private List<GameObject> m_RelatedObjects = new List<GameObject>();
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
        /// Backlog/ Callstack or sth like that? Still changing....
        /// new lines wiht \n supposed to be supported but might not work out
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
    #region providing methods

    /// <summary>
    /// returns State that was sent along with this notification
    /// </summary>
    /// <returns>reported state</returns>
    public DummyStates.State GetState()
    {
        return m_state;
    }
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
            return "No additional information given";
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
    /// returns information about note in the following format:  [Roboy BodyPart]: [state]
    /// </summary>
    /// <returns> [bodypart name]: [state]</returns>
    public string GetBasicInfo()
    {
        if(m_bodyPart) 
        return m_bodyPart.name.ToString() + ":\t" + m_state.ToString();
        return "[General]:\t" + m_state.ToString();
    }

    /// <summary>
    /// returns gameobject which is concerned for this notification.
    /// If no Roboy part linked -> null is returned!
    /// </summary>
    /// <returns></returns>
    public GameObject GetConcernedRoboyPart()
    {
        if (!m_bodyPart)
        {
            Debug.Log("no Roboy body part found!");
        }
        return m_bodyPart;
    }
    #endregion

    /// <summary>
    /// Basic Notification constructor, creating note with specified type and state
    /// </summary>
    /// <param name="type">What type of message (warning, error ...)</param>
    /// <param name="state">What state is the notification depicting</param>
    /// <param name="objName">The name of the concerned body part</param>
    /// <param name="timeFrame">time this notification is valid in seconds</param>
    public void SetupNotification(DummyStates.MessageType type, DummyStates.State state, string objName, double timeFrame)
    {
        m_state = state;
        m_type = type;
        m_content = null;
        SetConcernedRoboyPart(objName);
        if (m_bodyPart)
            m_bodyPart.GetComponent<RoboyPart>().AddNotification(this);
        m_timestamp = DateTime.Now;
        m_timeFrame = timeFrame;
        //TODO: small time difference might occure
        //get rid of this notification after specified amount of time
        Invoke("DeleteNotification", (float)m_timeFrame);
    }
   
    /// <summary>
    /// Displays Halo around concerned GameObject. Colour is chosen according to notification type
    /// </summary>
    public void DisplayHalo()
    {
        if (m_type != DummyStates.MessageType.DEBUG)
            if (m_bodyPart)
                m_bodyPart.GetComponent<RoboyPart>().UpdateHalo();

    }

    /// <summary>
    /// removes this message's link to roboy. Deletes all related objects. Removes itself from database.
    /// This method is automatically called as soon as the notification is not valid anymore.
    /// It can also be called manually.
    /// </summary>
    public void DeleteNotification()
    {
        //TODO: check what happens if called manually before automatic destruction called as well 
        if (m_bodyPart)
        {
            GetConcernedRoboyPart().GetComponent<RoboyPart>().RemoveNotification(this);
            //adjust halo to new/updated list 
            GetConcernedRoboyPart().GetComponent<RoboyPart>().UpdateHalo();
        }
        //delete every element that is related/dependent on this notification
        foreach (GameObject obj in m_RelatedObjects)
        {
            Destroy(obj);
        }
        VRUILogic.Instance.RemoveNotification(this);
        //destroy script and connected gameObject
        Destroy(gameObject);
    }

    /// <summary>
    /// Each gameObject that calls this function will be deleted as soon as the Notification becomes outdated.
    /// Once called there's no turning back / unsubscribe. 
    /// </summary>
    /// <param name="obj">object to be destroyed when outdated notification</param>
    public void AddRelatedObject(GameObject obj)
    {
        if (obj)
            m_RelatedObjects.Add(obj);
    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// sets roboy part with name as given string
    /// </summary>
    /// <param name="s">Roboy body part name</param>
    private void SetConcernedRoboyPart(string s)
    {
        //TODO find roboy body part according to obj id
        if (s.Equals(string.Empty))
        {
            Debug.Log("[Notification] No body part to attach notification to!");
        }
        m_bodyPart = GameObject.Find(s);
    }
    #endregion
}
