using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;

/// <summary>
/// Handles the ROSBridge connection. Adds all ROS components of each ROSObject in the scene. You need one object of this in each scene where you have ROS actors.
/// </summary>
public class ROSBridge : Singleton<ROSBridge>
{
    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// The IP address of the roscore running on the other side of the ROSBridge.
    /// </summary>
    public string ROSCoreIP = "";

    /// <summary>
    /// Port of the ROSBridge.
    /// </summary>
    public int Port = 9090;
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// ROS websocket connection.
    /// </summary>
    private ROSBridgeWebSocketConnection m_ROS = null;

    /// <summary>
    /// Is ROS initialized?
    /// </summary>
    private bool m_ROSInitialized = false;

    /// <summary>
    /// List of all active ROSObjects.
    /// </summary>
    private List<ROSObject> m_ROSObjects = new List<ROSObject>();

    /// <summary>
    /// List so we can add ROSObjects also when the connection is not established yet and add the objects as soon as the connection is established.
    /// </summary>
    private List<ROSObject> m_ROSObjectsToAdd = new List<ROSObject>();
    #endregion

    #region UNITY_MONOBEHAVIOUR
    /// <summary>
    /// Initializes the ROS websocket connection.
    /// </summary>
    private void Awake()
    {
        m_ROS = new ROSBridgeWebSocketConnection("ws://" + ROSCoreIP, Port);

        if (m_ROS != null)
        {
            m_ROS.Connect();
            m_ROSInitialized = true;
        }
    }

    /// <summary>
    /// Run ROSBridge if initialized.
    /// Announces newly registered Publishers / subscribers
    /// </summary>
    void Update()
    {
        if (!m_ROSInitialized || m_ROS == null)
            return;

        m_ROS.Render();
        CheckToDoList();
    }

    /// <summary>
    /// Disconnect from the simulation when Unity is not running, remove all publishers & subscribers.
    /// </summary>
    private void OnApplicationQuit()
    {
        Debug.Log("[ROSManager] Disconnecting from ROS");
        if (m_ROSInitialized)
        {
            m_ROS.Disconnect();
            m_ROSInitialized = false;
        }
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// 
    /// </summary>
    /// <param name="topic"></param>
    /// <param name="msg"></param>
    public void Publish(string topic, ROSBridgeMsg msg)
    {
        //Debug.Log("[ROSBridge Message] Publishing some message on topic" + topic);
        m_ROS.Publish(topic, msg);
    }

    public void CallService(string service, string args)
    {
        m_ROS.CallService(service, args);
    }

    /// <summary>
    /// Adds a ROS Actor aka. Publisher, Subscriber, Service to the ROSBridge. 
    /// Components will be checked and announced as soon as connection is established
    /// </summary>
    /// <param name="rosObject"></param>
    public void AddROSActor(ROSObject rosObject)
    {
        // add the rosobjects to "TODO list" so we can add them later
        m_ROSObjectsToAdd.Add(rosObject);
        return;
    }

    /// <summary>
    /// Removes a ROS Actor. Is useful so objects can disconnect from the ROSBridge if they are not active and do not need to be to connected again.
    /// </summary>
    /// <param name="rosObject"></param>
    public void RemoveROSActor(ROSObject rosObject)
    {
        Debug.Log("[ROSBridge] Removing ROS Actor");
        //IF not added yet but on "todo list"
        if (m_ROSObjectsToAdd.Contains(rosObject))
        {
            m_ROSObjectsToAdd.Remove(rosObject);
        }

        if (!m_ROSInitialized)
        {
            return;
        }

        if (!m_ROSObjects.Contains(rosObject))
            return;

        var subscribers = rosObject.GetComponents<ROSBridgeSubscriber>();
        var publishers = rosObject.GetComponents<ROSBridgePublisher>();
        var services = rosObject.GetComponents<ROSBridgeService>();

        foreach (var sub in subscribers)
            m_ROS.RemoveSubscriber(sub.GetType());

        foreach (var pub in publishers)
            m_ROS.RemovePublisher(pub.GetType());

        foreach (var serv in services)
            m_ROS.RemoveServiceResponse(serv.GetType());

        m_ROSObjects.Remove(rosObject);
    }

    /// <summary>
    /// Returns whether connection to ROS bridge established
    /// </summary>
    /// <returns></returns>
    public bool IsConnected()
    {
        return m_ROS != null && m_ROSInitialized;
    }
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// Goes through the list of ROSObject to add (which are not yet added), and announces & advertises all 
    /// Only goes through list when websocket is running, otherwise no announcements can be made
    /// </summary>
    private void CheckToDoList()
    {
        // add all cached ROSObjects and delete them from the "TODO" list
        if (m_ROSInitialized && m_ROSObjectsToAdd.Count > 0)
        {
            foreach (var rosObject in m_ROSObjectsToAdd)
            {
                if (m_ROSObjects.Contains(rosObject))
                    return;

                var subscribers = rosObject.GetComponents<ROSBridgeSubscriber>();
                var publishers = rosObject.GetComponents<ROSBridgePublisher>();
                var services = rosObject.GetComponents<ROSBridgeService>();

                foreach (var sub in subscribers)
                {
                    m_ROS.AddSubscriber(sub.GetType());
                    //Debug.Log("[ROSBridge]: adding subscriber " + sub.name);
                }

                foreach (var pub in publishers)
                {
                    m_ROS.AddPublisher(pub.GetType());
                    //Debug.Log("[ROSBridge]: adding publisher " + pub.name);
                }
                foreach (var serv in services)
                {
                    m_ROS.AddServiceResponse(serv.GetType());
                    //Debug.Log("[ROSBridge]: adding service " + serv.name);
                }
                //add to list of established / known objects
                m_ROSObjects.Add(rosObject);
            }
            m_ROSObjectsToAdd.Clear();
        }
    }
    #endregion
}