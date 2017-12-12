using UnityEngine;
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.custom_msgs;

/// <summary>
/// Roboymanager has the task to adjust roboys state depending on the ROS messages.
/// In summary it does the following:
///
///     -# receive pose messages to adjust roboy pose.
///     -# subscribe to the external force event and forward the message to the simulation.
///     -# send a service call for a world reset.
///     -# FUTURE: receive motor msg and forward it to the according motors.
/// </summary>
public class RoboyManager : Singleton<RoboyManager>
{

    #region PUBLIC_MEMBER_VARIABLES

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
                Debug.LogWarning("[RoboyManager] Roboy could not be found! Returning null!");
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

    private Dictionary<string, Transform> m_Roboys = new Dictionary<string, Transform>();

    /// <summary>
    /// Pose message of roboy in ou r build in class
    /// </summary>
    private RoboyPoseMsg m_RoboyPoseMessage;

    /// <summary>
    /// Dictionary with all roboyparts, used to adjust pose and motor values
    /// </summary>
    private Dictionary<string, RoboyPart> m_RoboyParts
        = new Dictionary<string, RoboyPart>();

    /// <summary>
    /// list of roboy_parts for each roboy (name maps to his parts in a dictionary)
    /// </summary>
    private Dictionary<string, Dictionary<string, RoboyPart>> m_RoboyPartsList = new Dictionary<string, Dictionary<string, RoboyPart>>();

    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initialize ROSBridge and roboy parts
    /// </summary>
    private void Awake()
    {
        //srarches for all roboys in the scene and all their parts
        getRoboy();
        InitializeRoboyParts();
    }

    /// <summary>
    /// Run ROSBridge, Reset ROS simulation if Key R is pressed
    /// </summary>
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
            ResetSimulation();
    }

    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS
    public void AddRoboy(Transform roboy)
    {
        if (!m_Roboys.ContainsKey(roboy.name))
        {
            Debug.Log("[RoboyManager] Adding new roboy: " + roboy.name);
            m_Roboys.Add(roboy.name, roboy);
            getRoboyParts(roboy.name);
        }
    }

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
        AdjustPose(msg.Name, msg);

        //Use additional data to adjust motor values

    }

    /// <summary>
    /// Sends a message to the simulation to apply an external force at a certain position. 
    /// The position is transformed from Unity to Gazebo space here.
    /// </summary>
    /// <param name="roboyPart">The roboypart where the force should be applied.</param>
    /// <param name="position">The relative position of the force to the roboypart.</param>
    /// <param name="force">The direction and the amount of force relative to roboypart.</param>
    /// <param name="duration">The duration for which the force should be applied.</param>
    public void ReceiveExternalForce(RoboyPart roboyPart, Vector3 position, Vector3 force, int duration)
    {
        ROSBridgeLib.custom_msgs.ExternalForceMsg msg;
        if (roboyPart == null)
        { //TODO fix this (shouldit be sent at all?) if so -> empty string
            msg = new ROSBridgeLib.custom_msgs.ExternalForceMsg("Roboys-Unicorn-horn", GazeboUtility.UnityPositionToGazebo(position), GazeboUtility.UnityPositionToGazebo(force), duration);
        }
        else
        {
            msg = new ROSBridgeLib.custom_msgs.ExternalForceMsg(roboyPart.gameObject.name, GazeboUtility.UnityPositionToGazebo(position), GazeboUtility.UnityPositionToGazebo(force), duration);
        }
        ROSBridge.Instance.Publish(RoboyForcePublisher.GetMessageTopic(), msg);

        //Debug.Log(msg.ToYAMLString());
    }

    public void ResetSimulation()
    {
        ROSBridge.Instance.CallService("/roboy/reset_world", "");
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    /// <summary>
    /// Test function to draw tendons. For now draws only random lines. TEMPLATE!
    /// </summary>
    private void drawTendons()
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
    private void AdjustPose(string name, RoboyPoseMsg msg)
    {
        IDictionary<string, RoboyPart> dic;
        m_RoboyPoseMessage = msg;

        Dictionary<string, float> xPositionsDictionary = m_RoboyPoseMessage.XDic;
        Dictionary<string, float> yPositionsDictionary = m_RoboyPoseMessage.YDic;
        Dictionary<string, float> zPositionsDictionary = m_RoboyPoseMessage.ZDic;

        Dictionary<string, float> qxRotationsDictionary = m_RoboyPoseMessage.QxDic;
        Dictionary<string, float> qyRotationsDictionary = m_RoboyPoseMessage.QyDic;
        Dictionary<string, float> qzRotationsDictionary = m_RoboyPoseMessage.QzDic;
        Dictionary<string, float> qwRotationsDictionary = m_RoboyPoseMessage.QwDic;

        if (name == null)
        {
            Debug.Log("[RoboyManager]  Using default roboy since no other name specified.");
            name = m_Roboy.name;
        }
        try
        {
            dic = m_RoboyPartsList[m_Roboy.name];
        }
        catch
        {

            return;
        }
        foreach (KeyValuePair<string, RoboyPart> roboyPart in m_RoboyPartsList[name])
        {
            string index = roboyPart.Key;
            try
            {
                Vector3 originPosition = new Vector3(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index]);
                Quaternion originRotation = new Quaternion(qxRotationsDictionary[index], qyRotationsDictionary[index], qzRotationsDictionary[index], qwRotationsDictionary[index]);

                roboyPart.Value.transform.localPosition = GazeboUtility.GazeboPositionToUnity(originPosition);
                roboyPart.Value.transform.localRotation = GazeboUtility.GazeboRotationToUnity(originRotation);
            }
            catch
            {
                Debug.Log("[RoboyManager] Couldn't find body part named: " + index);
            }
        }
    }

    /// <summary>
    /// Searches for all roboys via the "Roboy" tag.
    /// </summary>
    private void getRoboy()
    {
        GameObject roboy = GameObject.FindGameObjectWithTag("Roboy");
        if (roboy)
        {
            m_Roboy = roboy.transform;
            AddRoboy(roboy.transform);
        }
        else
        {
            Debug.LogWarning("[RoboyManager] Roboy could not be found!");
            return;
        }
    }

    /// <summary>
    /// Searches for all roboy parts for the specified roboy(s).
    /// </summary>
    private void getRoboyParts(string name)
    {
        if (!m_Roboys.ContainsKey(name))
        {
            return;
        }
        m_RoboyPartsList.Add(name, new Dictionary<string, RoboyPart>());
        foreach (Transform trans in m_Roboys[name])
        {
            foreach (Transform t in trans.GetComponentsInChildren<Transform>())
            {
                if (t == null | !t.CompareTag("RoboyPart"))
                    continue;
                m_RoboyParts.Add(t.name, t.GetComponent<RoboyPart>());
                m_RoboyPartsList[name].Add(t.name, t.GetComponent<RoboyPart>());
                //m_RoboyParts.Add(t.name, t.GetComponent<RoboyPart>());
            }
        }
    }
    #endregion //PRIVATE_METHODS
}