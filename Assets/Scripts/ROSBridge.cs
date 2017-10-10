using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;

/// <summary>
/// Handles the ROSBridge connection. Adds all ROS components of each ROSObject in the scene. You need one object of this in each scene where you have ROS actors.
/// </summary>
public class ROSBridge : Singleton<ROSBridge> {

    /// <summary>
    /// The IP address of the roscore running on the other side of the ROSBridge.
    /// </summary>
    public string ROSCoreIP = "";

    /// <summary>
    /// Port of the ROSBridge.
    /// </summary>
    public int Port = 9090;

    ///// <summary>
    ///// Public property for other classes to the ros websocket.
    ///// </summary>
    //public ROSBridgeWebSocketConnection ROS { get { return m_ROS; } }

    /// <summary>
    /// Public property of all active ROSObjects in the scene.
    /// </summary>
    public List<ROSObject> ROSObjects { get { return m_ROSObjects; } }

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

    /// <summary>
    /// Initializes the ROS websocket connection and searches for all ROSObjects in the scene.
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

    public void Publish(string topic, ROSBridgeMsg msg)
    {
        m_ROS.Publish(topic, msg);
    }

    public void CallService(string service, string args)
    {
        m_ROS.CallService(service, args);
    }

    /// <summary>
    /// Adds a ROS Actor aka. Publisher, Subscriber, Service to the ROSBridge. Is public so a ROSObject can invoke it after Awake.
    /// </summary>
    /// <param name="rosObject"></param>
    public void AddROSActor(ROSObject rosObject)
    {
        if (!m_ROSInitialized)
        {
            // add the cached rosobjects so we can add them later
            m_ROSObjectsToAdd.Add(rosObject);
            return;
        }

        if (m_ROSObjects.Contains(rosObject))
            return;

        var subscribers = rosObject.GetComponents<ROSBridgeSubscriber>();
        var publishers = rosObject.GetComponents<ROSBridgePublisher>();
        var services = rosObject.GetComponents<ROSBridgeService>();
        
        foreach (var sub in subscribers)
            m_ROS.AddSubscriber(sub.GetType());

        foreach (var pub in publishers)
            m_ROS.AddPublisher(pub.GetType());

        foreach (var serv in services)
            m_ROS.AddServiceResponse(serv.GetType());

        m_ROSObjects.Add(rosObject);
    }

    /// <summary>
    /// Removes a ROS Actor. Is useful so objects can disconnect from the ROSBridge if they are not active and do not need to be to connected again.
    /// </summary>
    /// <param name="rosObject"></param>
    public void RemoveROSActor(ROSObject rosObject)
    {
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
    /// Run ROSBridge if initialized.
    /// </summary>
    private void Update()
    {
        if (!m_ROSInitialized)
            return;

        m_ROS.Render();

        // add all cached ROSObjects and delete them from the cached list
        if (m_ROSObjectsToAdd.Count > 0 )
        {
            foreach (var rosObject in m_ROSObjectsToAdd)
            {
                AddROSActor(rosObject);
            }
            m_ROSObjectsToAdd.Clear();
        }
    }

    /// <summary>
    /// Disconnect from the simulation when Unity is not running.
    /// </summary>
    private void OnApplicationQuit()
    {
        if (m_ROSInitialized)
        {
            m_ROS.Disconnect();
            m_ROSInitialized = false;
        }
    }
}
