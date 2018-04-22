using UnityEngine;
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.custom_msgs;

/// <summary>
/// Roboymanager has the task to adjust roboys state depending on the ROS messages.
/// In summary it does the following:
///
///     -# receive pose messages to adjust roboy pose.
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
    /// Pose message of roboy in our build in class
    /// </summary>
    private RoboyPoseMsg m_RoboyPoseMessage;

    /// <summary>
    /// Dictionary with all roboyparts, used to adjust pose and motor values
    /// </summary>
    private Dictionary<string, RoboyPart> m_RoboyParts = new Dictionary<string, RoboyPart>();

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
        LocateRoboy();
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
            LocateRoboyParts(roboy.name);
        }
    }

    /// <summary>
    /// Initializes the roboy parts with a random count of motors => WILL BE CHANGED IN THE FUTURE, for now just a template
    /// </summary>
    public void InitializeRoboyParts()
    {
        foreach (var roboyPart in m_RoboyParts)
        {
            int motorCount = Random.Range(1, 8);
            roboyPart.Value.Initialize(motorCount);
        }
    }

    /// <summary>
    /// Main function to receive messages from ROSBridge. Adjusts the roboy pose
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveMessage(RoboyPoseMsg msg)
    {
        //DEBUG AID: for msg count per second: Debug.Log("[RoboyManager] " + (int)Time.realtimeSinceStartup + "adjusting pose ");
        AdjustPose(msg.Name, msg);
    }

    /// <summary>
    /// Sends a message to the simulation to apply an external force at a certain position. 
    /// The position is transformed from Unity to Gazebo space here.
    /// </summary>
    /// <param name="roboyPart">The roboypart where the force should be applied.</param>
    /// <param name="position">The RELATIVE position of the force to the roboypart.</param>
    /// <param name="force">The RELATIVE direction and the amount of force with respect to the roboypart.</param>
    /// <param name="duration">The duration for which the force should be applied (in millisecs).</param>
    public void SendExternalForce(RoboyPart roboyPart, Vector3 position, Vector3 force, int duration)
    {
        ExternalForceMsg msg;
        if (roboyPart != null)
        {
            //make sure to not send faulty values 
            if (float.IsInfinity(force.x) || float.IsInfinity(force.y) || float.IsInfinity(force.z))
                return;
            msg = new ExternalForceMsg(roboyPart.gameObject.name, position, force, duration, true);
            ROSBridge.Instance.Publish(RoboyForcePublisher.GetMessageTopic(), msg);
        }
    }

    public void ResetSimulation()
    {
        ROSBridge.Instance.CallService("/roboy/reset_world", "");
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    /// <summary>
    /// Adjusts roboy pose for all parts with the values from the simulation.
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void AdjustPose(string name, RoboyPoseMsg msg)
    {
        IDictionary<string, RoboyPart> dic;
        m_RoboyPoseMessage = msg;

        if (name == null)
        {
            //Debug.Log("[RoboyManager]  Using default roboy since no other name specified.");
            //TODO for now: no name specified for this message type anyways
            name = m_Roboy.name;
        }
        try
        {
            dic = m_RoboyPartsList[m_Roboy.name];
        }
        catch
        {
            Debug.Log("Could not find roboy with name  " + m_Roboy.name + " to which to apply the new pose");
            return;
        }

        //for each roboypart which was specified
        for (int i = 0; i < msg.linkNames.Count; i++)
        {
            RoboyPart part;
            //if roboy part exists
            if (dic.TryGetValue(msg.linkNames[i], out part))
            {
                part.transform.position = msg.positions[i];
                part.transform.rotation = msg.rotations[i];
            }
            else
            {
                Debug.Log("[RoboyManager] Could not find link named " + msg.linkNames[i] +
                    "\nMaybe this part could not be found due to a missing tag?");
            }
        }
    }

    /// <summary>
    /// Searches for all roboys via the "Roboy" tag.
    /// </summary>
    private void LocateRoboy()
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
    private void LocateRoboyParts(string name)
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