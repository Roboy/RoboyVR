using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This class created examplary data.
/// This includes 4 tendons and a continuous notification feed.
/// </summary>
public class ExampleFunctions : MonoBehaviour
{

    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES   
    /// <summary>
    /// Reference needed to continuously rotate arm
    /// </summary>
    private GameObject m_Arm;

    /// <summary>
    /// Boolean managing continuous notification spawn (for coroutine management)
    /// </summary>
    private bool m_Testing = false;

    /// <summary>
    /// Offset to position the tendons at the right spot, applied constantly
    /// </summary>
    [SerializeField]
    private Vector3 m_Offset;

    /// <summary>
    /// Rference to all points so that offset can be applied
    /// </summary>
    private List<Vector3> m_AllPoints = new List<Vector3>();

    /// <summary>
    /// Old offset to cheeck for changes and only apply it once
    /// </summary>
    private Vector3 m_oldOffset;

    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// initializes tendons and it relations.
    /// Tendons dependent on current model (Names)!
    /// </summary>
    void Start()
    {
        //TENDON EXAMPLE
        /*
        Vector3[] points1 = { new Vector3(0.242f, 0.355f, -0.034f), //upper arm left
                              new Vector3(0.2358f, 0.4768f, -0.024f),}; //torso

        Vector3[] points2 = { new Vector3(0.18f, 0.342f, -0.034f), //upper arm left
                              new Vector3(0.1771f, 0.4585f, 0) }; //torso

        Vector3[] points3 = { new Vector3(0.2337f, 0.258f, -0.0643f), //upper arm left
                              new Vector3(0.2204f, 0.1817f, -0.0833f), //lower arm left
                              new Vector3(0.2109f, 0.1362f, -0.1178f), //lower arm left
                              new Vector3(0.2109f, 0.1193f, -0.134f)}; //lower arm left

        Vector3[] points4 = { new Vector3(0.1814f, 0.274f, -0.047f), //upper arm left
                              new Vector3(0.1814f, 0.164f, -0.072f), //lower arm left
                              new Vector3(0.1814f, 0.1287f, -0.102f), //lower arm left
                              new Vector3(0.1814f, 0.0947f, -0.157f)}; //lower arm left
        
        string[] names = { "upper_arm_left", "torso" };

        string[] names3 = { "upper_arm_left", "lower_arm_left", "lower_arm_left", "lower_arm_left" };
        */
        Vector3[] points1 = { new Vector3(0.2559f, 0.8919f, -0.0379f), //upper arm left
                              new Vector3(0.2716f, 1.0055f, -0.0545f),}; //torso

        Vector3[] points2 = { new Vector3(0.1999f, 0.9031f, -0.046f), //upper arm left
                              new Vector3(0.1999f, 1.0024f, -0.046f) }; //torso

        Vector3[] points3 = { new Vector3(0.1999f, 0.8103f, -0.0557f), //upper arm left
                              new Vector3(0.2178f, 0.7198f, -0.1119f), //lower arm left
                              new Vector3(0.22383f, 0.72199f, -0.1541f)}; //lower arm left

        Vector3[] points4 = { new Vector3(0.267f, 0.8133f, -0.0394f), //upper arm left
                              new Vector3(0.2516f, 0.7278f, -0.0891f), //lower arm left
                              new Vector3(0.2516f, 0.7374f, -0.1462f)}; //lower arm left

        string[] names = { "upper_arm_left", "torso" };

        string[] names3 = { "upper_arm_left", "lower_arm_left", "lower_arm_left" };
        VRUILogic.Instance.AddTendon(0, points1, names, 1f);
        VRUILogic.Instance.AddTendon(1, points2, names, 1f);
        VRUILogic.Instance.AddTendon(2, points3, names3, 1f);
        VRUILogic.Instance.AddTendon(3, points4, names3, 1f);
        // m_Arm = VRUILogic.Instance.GetBodyPart("lower_arm_left");

        foreach(var point in points1){
            m_AllPoints.Add(point);
        }
        foreach (var point in points2)
        {
            m_AllPoints.Add(point);
        }

        foreach (var point in points3)
        {
            m_AllPoints.Add(point);
        }
        foreach (var point in points4)
        {
            m_AllPoints.Add(point);
        }
        // update initial position of tendon wirepoints, after that, they should be automatically adjusted due to child / parent relation
        UpdatOffset();

    }

    /// <summary>
    /// Called once every frame, updates tendon color,arm position and keeps notification feed going
    /// </summary>
    void Update()
    {
        //TENDON EXAMPLE
        float x = Mathf.Cos(Time.realtimeSinceStartup * 8 / (2 * 3.14159f)) * 0.5f + 0.5f; //amplitude: 0-1, period length = 8
        VRUILogic.Instance.UpdateTendon(0, x);
        VRUILogic.Instance.UpdateTendon(1, x);
        VRUILogic.Instance.UpdateTendon(2, 1 - x);
        VRUILogic.Instance.UpdateTendon(3, 1 - x);
        
        //Rotates arm for which the tendons are defined -> shows that they move along. 
        //Does not work if a connection to ROS established and the pose is being published there -> overrides this 
        if (m_Arm && !ROSBridge.Instance.IsConnected())
        {
            Vector3 temp = m_Arm.transform.position;
            Vector3 offset = new Vector3();
            float angleparts = 4 / Time.deltaTime; // how many parts fit into 4 secs
            float time = Time.time;
            if (((int)time / 4) % 2 == 0) //0-4 8-12 .... for first 4 secs turn arm outwards
            {
                m_Arm.transform.RotateAround(temp + offset, Vector3.up, -90 / angleparts);
            }
            else //4-8, 12-16 .... for next 4 secs turn arm back to original pos
            {
                m_Arm.transform.RotateAround(temp + offset, Vector3.up, 90 / angleparts);
            }
        }
        //NOTIFICATIONS
        //test notifications display
        if (!m_Testing)
        {
            m_Testing = true;
            StartCoroutine(test());
        }

    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// applies new offset relative to initial position ( not the current one)
    /// </summary>
    private void UpdatOffset()
    {
        if (m_oldOffset.Equals(m_Offset)) return;
        Debug.Log("[ExampleFunctions] Apply Roboy's position to tendon positions");
        Vector3 deltaOffset = m_Offset - m_oldOffset;
        VRUILogic.Instance.ApplyTendonOffset(0, deltaOffset);
        VRUILogic.Instance.ApplyTendonOffset(1, deltaOffset);
        VRUILogic.Instance.ApplyTendonOffset(2, deltaOffset);
        VRUILogic.Instance.ApplyTendonOffset(3, deltaOffset);
        m_oldOffset = m_Offset;
    }
    /// <summary>
    /// used for testing of notifications display.
    /// Automatically spawns new notifications.
    /// </summary>
    /// <returns></returns>
    private IEnumerator test()
    {
        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, "oberarm_right", 3);
        //Debug.Log("[ExampleFunctions] Added Warning");

        yield return new WaitForSeconds(4);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, "hip", 5);
        //Debug.Log("[ExampleFunctions] Added Error");

        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, "foot_left", 5);
        //Debug.Log("[ExampleFunctions] Added Debug");

        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, "head", 5f);
        // Debug.Log("[ExampleFunctions] Added debug");
        m_Testing = false;
    }
    #endregion
}

