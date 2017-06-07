using UnityEngine;
using System.Collections.Generic;
using ROSBridgeLib;

/// <summary>
/// Roboymanager has different tasks:
///
/// <b>- Run ROS:</b>
///     -# Connect to the simulation.
///     -# Add subscriber to the pose.
///     -# Add publisher for external force.
///     -# Add service response for world reset.
///
/// <b>- Receive and send ROS messages:</b>
///     -# receive pose msg to adjust roboy pose.
///     -# subscribe to external force event and send msg to simulation.
///     -# send service call for world reset.
///     -# FUTURE: receive motor msg and forward it to the according motors.
/// </summary>
public class RoboyManager : Singleton<RoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES

    /// <summary>
    /// The IP address of the VM or the machine where the simulation is running
    /// </summary>
    public string VM_IP = "";
    
    /// <summary>
    /// Public variable so that all classes can access the roboy object.
    /// </summary>
    public Transform Roboy
    {
        get
        {
            if (m_Roboy != null)
                return m_Roboy;

            m_Roboy = GameObject.FindGameObjectWithTag("Roboy").transform;
            if (m_Roboy != null)
                return m_Roboy;
            else
            {
                Debug.LogWarning("Roboy could not be found! Returning null!");
                return null;
            }
        }
    }

    /// <summary>
    /// Public variable for the dictionary with all roboyparts, used to adjust pose and motor values
    /// </summary>
    public Dictionary<string, RoboyPart> RoboyParts
    {
        get
        {
            return m_RoboyParts;
        }
    }

    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Transform of roboy with all roboy parts as child objects
    /// </summary>
    [SerializeField]
    private Transform m_Roboy;
    /// <summary>
    /// ROSBridge websocket
    /// </summary>
    private ROSBridgeWebSocketConnection m_Ros = null;
    /// <summary>
    /// Pose message of roboy in our build in class
    /// </summary>
    private RoboyPoseMsg m_RoboyPoseMessage;

    /// <summary>
    /// Variable to check if the ROS connection is working!
    /// </summary>
    private bool m_ROSInitialized = false;

    /// <summary>
    /// Dictionary with all roboyparts, used to adjust pose and motor values
    /// </summary>
    private Dictionary<string, RoboyPart> m_RoboyParts
        = new Dictionary<string, RoboyPart>();

    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initialize ROSBridge and roboy parts
    /// </summary>
    void Awake()
    {
        if (string.IsNullOrEmpty(VM_IP))
            return;

        m_Ros = new ROSBridgeWebSocketConnection("ws://" + VM_IP, 9090);

        // DOES NOT WORK! m_Ros is never null if you call the Constructor, WAIT UNTILL SIMON IMPLEMENTS UDP BROADCAST WITH ROS CONFIGURATION, GET THE IP ADDRESS FROM THE BROADCAST.
        if (m_Ros != null)
        {
            m_Ros.AddSubscriber(typeof(RoboyPoseSubscriber));
            m_Ros.AddServiceResponse(typeof(RoboyServiceResponse));
            m_Ros.AddPublisher(typeof(RoboyPosePublisher));
            m_Ros.Connect();
            m_ROSInitialized = true;

            Debug.Log("ROS successfully initialized!");
        }
        else
        {
            Debug.LogWarning("ROS could not be initialized!");
        }

        getRoboy();

        getRoboyParts();

        InitializeRoboyParts();
    }

    /// <summary>
    /// Run ROSBridge
    /// </summary>
    void Update()
    {
        if (m_ROSInitialized)
        {
            m_Ros.Render();

            if (Input.GetKeyDown(KeyCode.R))
                ResetSimulation();
        }
    }

    /// <summary>
    /// Disconnect from the simulation when Unity is not running.
    /// </summary>
    void OnApplicationQuit()
    {
        if (!m_ROSInitialized)
            m_Ros.Disconnect();
    }
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    /// <summary>
    /// Initializes the roboy parts with a random count of motors => WILL BE CHANGED IN THE FUTURE, for now just a template
    /// </summary>
    public void InitializeRoboyParts()
    {
        foreach (var roboyPart in m_RoboyParts)
        {
            int motorCount = UnityEngine.Random.Range(1, 8);
            roboyPart.Value.Initialize(motorCount);
        }
    }

    /// <summary>
    /// Main function to receive messages from ROSBridge. Adjusts the roboy pose and the motors values (future).
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveMessage(RoboyPoseMsg msg)
    {
        //Debug.Log("Received message");
        adjustPose(msg);
        
        //Use additional data to adjust motor values

    }

    /// <summary>
    /// Sends a message to the simulation to apply an external force at a certain position.
    /// </summary>
    /// <param name="roboyPart">The roboypart where the force should be applied.</param>
    /// <param name="position">The relative position of the force to the roboypart.</param>
    /// <param name="force">The direction and the amount of force relative to roboypart.</param>
    /// <param name="duration">The duration for which the force should be applied.</param>
    public void ReceiveExternalForce(RoboyPart roboyPart, Vector3 position, Vector3 force, int duration)
    {
        ROSBridgeLib.custom_msgs.ExternalForceMsg msg = 
            new ROSBridgeLib.custom_msgs.ExternalForceMsg(roboyPart.gameObject.name, unityPositionToGazebo(position), unityPositionToGazebo(force), duration);

        m_Ros.Publish(RoboyPosePublisher.GetMessageTopic(), msg);

        //Debug.Log(msg.ToYAMLString());
    }

    public void ResetSimulation()
    {
        if (!m_ROSInitialized)
        {
            Debug.LogWarning("Cannot reset simulation as ROS is not running!");
            return;
        }
     
        m_Ros.CallService("/roboy/reset_world", "");
    }

    #region Convert function from gazebo to unity and vice versa.
    /// <summary>
    /// Converts a quaternion in gazebo coordinate frame to unity coordinate frame.
    /// </summary>
    /// <param name="gazeboRot">Quaternion in gazebo coordinate frame.</param>
    /// <returns>Quaternion in unity coordinate frame.</returns>
    Quaternion gazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = new Quaternion(-gazeboRot.x, -gazeboRot.z, -gazeboRot.y, gazeboRot.w);

        Quaternion finalRot = tempRot*rotZ*rotX;

        return finalRot;
    }

    /// <summary>
    /// Converts a vector in gazebo coordinate frame to unity coordinate frame.
    /// </summary>
    /// <param name="gazeboPos">Vector in gazebo coordinate frame.</param>
    /// <returns>Vector in unity coordinate frame.</returns>
    Vector3 gazeboPositionToUnity(Vector3 gazeboPos)
    {
        return new Vector3(gazeboPos.x, gazeboPos.z, gazeboPos.y);
    }

    /// <summary>
    /// Converts a vector in unity coordinate frame to gazebo coordinate frame.
    /// </summary>
    /// <param name="unityPos">Vector in unity coordinate frame.</param>
    /// <returns>Vector in gazebo coordinate frame.</returns>
    Vector3 unityPositionToGazebo(Vector3 unityPos)
    {
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    /// <summary>
    /// Converts a quaternion in unity coordinate frame to gazebo coordinate frame.
    /// </summary>
    /// <param name="unityRot">Quaternion in unity coordinate frame.</param>
    /// <returns>Quaternion in gazebo coordinate frame.</returns>
    Quaternion unityRotationToGazebo(Quaternion unityRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = unityRot*rotX*rotZ;

        Quaternion finalRot = new Quaternion(-tempRot.x, -tempRot.z, -tempRot.y, tempRot.w);

        return finalRot;
    }
    #endregion
    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    /// <summary>
    /// Test function to draw tendons. For now draws only random lines. TEMPLATE!
    /// </summary>
    void drawTendons()
    {
        Dictionary<int, List<Vector3>> tendonsDictionary = new Dictionary<int, List<Vector3>>();
        int tendonsCount = 5;
        int tendonsLength = UnityEngine.Random.Range(2, 10);

        for (int i = 0; i < tendonsCount; i++)
        {
            List<Vector3> lv = new List<Vector3>();
            for (int j = 0; j < tendonsLength; j++)
            {
                Vector3 position = new Vector3(UnityEngine.Random.Range(-10.0f, 10.0f), 0, UnityEngine.Random.Range(-10.0f, 10.0f));
                lv.Add(position);
            }
            tendonsDictionary.Add(i, lv);
            tendonsLength = UnityEngine.Random.Range(2, 10);
        }

        foreach (KeyValuePair<int, List<Vector3>> t in tendonsDictionary)
        {
            //Get number of points for linerenderer
            int points = t.Value.Count;

            GameObject g = new GameObject();
            g.AddComponent<LineRenderer>();
            LineRenderer lr = g.GetComponent<LineRenderer>();
            lr.positionCount = points - 1;
            lr.SetPositions(t.Value.ToArray());
            lr.startWidth = lr.endWidth = 0.1f;
        }
    }

    /// <summary>
    /// Adjusts roboy pose for all parts with the values from the simulation.
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    void adjustPose(RoboyPoseMsg msg)
    {
        m_RoboyPoseMessage = msg;

        Dictionary<string, float> xPositionsDictionary = m_RoboyPoseMessage.XDic;
        Dictionary<string, float> yPositionsDictionary = m_RoboyPoseMessage.YDic;
        Dictionary<string, float> zPositionsDictionary = m_RoboyPoseMessage.ZDic;

        Dictionary<string, float> qxRotationsDictionary = m_RoboyPoseMessage.QxDic;
        Dictionary<string, float> qyRotationsDictionary = m_RoboyPoseMessage.QyDic;
        Dictionary<string, float> qzRotationsDictionary = m_RoboyPoseMessage.QzDic;
        Dictionary<string, float> qwRotationsDictionary = m_RoboyPoseMessage.QwDic;

        foreach (KeyValuePair<string, RoboyPart> roboyPart in m_RoboyParts)
        {
            string index = roboyPart.Key;
            Vector3 originPosition = new Vector3(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index]);
            Quaternion originRotation = new Quaternion(qxRotationsDictionary[index], qyRotationsDictionary[index], qzRotationsDictionary[index], qwRotationsDictionary[index]);

            roboyPart.Value.transform.localPosition = gazeboPositionToUnity(originPosition);
            roboyPart.Value.transform.localRotation = gazeboRotationToUnity(originRotation);
        }
    }

    void getRoboy()
    {
        if ((m_Roboy = GameObject.FindGameObjectWithTag("Roboy").transform) == null)
        {
            Debug.LogWarning("Roboy could not be found!");
            return;
        }
    }

    void getRoboyParts() {

        if (m_Roboy == null)
        {
            getRoboy();
        }

        foreach (Transform t in m_Roboy)
        {
            if (t == null | !t.CompareTag("RoboyPart"))
                continue;
            m_RoboyParts.Add(t.name, t.GetComponent<RoboyPart>());
        }
    }



    #endregion //PRIVATE_METHODS
}
