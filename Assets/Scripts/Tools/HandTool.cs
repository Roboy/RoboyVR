using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;

/// <summary>
/// Tool to push Roboy around and apply forces via direct contact.
/// </summary>
public class HandTool : ControllerTool {

    [SerializeField]
    Mesh m_RightHandMesh;

    [SerializeField]
    Mesh m_LeftHandMesh;

    private RoboyHandsPublisher m_RoboyHandsPublisher;

    private MeshFilter m_MeshFilter;

    private bool m_IsLeft = false;

    /// <summary>
    /// Get which model to use for hand: right or left.
    /// </summary>
    private IEnumerator Start()
    {
        m_MeshFilter = GetComponent<MeshFilter>();

        while (!m_Initialized)
            yield return null;

        int rightIndex = SteamVR_Controller.GetDeviceIndex(SteamVR_Controller.DeviceRelation.Rightmost);
        int leftIndex = SteamVR_Controller.GetDeviceIndex(SteamVR_Controller.DeviceRelation.Leftmost);

        SteamVR_Controller.Device rightDevice = SteamVR_Controller.Input(rightIndex);
        SteamVR_Controller.Device leftDevice = SteamVR_Controller.Input(leftIndex);

        if (rightDevice == m_SteamVRDevice)
        {
            m_MeshFilter.mesh = m_RightHandMesh;
            m_IsLeft = false;
        }
        else if (leftDevice == m_SteamVRDevice)
        {
            m_MeshFilter.mesh = m_LeftHandMesh;
            m_IsLeft = true;
        }
    }

    private void Update()
    {

        string linkName = (m_IsLeft) ? "left_hand" : "right_hand";
        List<string> linkNames = new List<string>();
        linkNames.Add(linkName);

        var xDic = new Dictionary<string, float>();
        var yDic = new Dictionary<string, float>();
        var zDic = new Dictionary<string, float>();
        var qxDic = new Dictionary<string, float>();
        var qyDic = new Dictionary<string, float>();
        var qzDic = new Dictionary<string, float>();
        var qwDic = new Dictionary<string, float>();

        Vector3 gazeboPosition = GazeboUtility.UnityPositionToGazebo(transform.position);
        Quaternion gazeboRotation = GazeboUtility.UnityRotationToGazebo(transform.rotation);

        //Vector3 gazeboPosition = transform.position;
        //Quaternion gazeboRotation = transform.rotation;

        xDic.Add(linkName, gazeboPosition.x);
        yDic.Add(linkName, gazeboPosition.y);
        zDic.Add(linkName, gazeboPosition.z);

        qxDic.Add(linkName, gazeboRotation.x);
        qyDic.Add(linkName, gazeboRotation.y);
        qzDic.Add(linkName, gazeboRotation.z);
        qwDic.Add(linkName, gazeboRotation.w);

        ROSBridgeLib.custom_msgs.RoboyPoseMsg msg = new ROSBridgeLib.custom_msgs.RoboyPoseMsg(linkNames, xDic, yDic, zDic, qxDic, qyDic, qzDic, qwDic);
        ROSBridge.Instance.Publish(RoboyHandsPublisher.GetMessageTopic(), msg);
    }
}
