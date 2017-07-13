using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Tool to push Roboy around and apply forces via direct contact.
/// </summary>
public class HandTool : ControllerTool {

    [SerializeField]
    Mesh m_RightHandMesh;

    [SerializeField]
    Mesh m_LeftHandMesh;

    private MeshFilter m_MeshFilter;

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
        }
        else if (leftDevice == m_SteamVRDevice)
        {
            m_MeshFilter.mesh = m_LeftHandMesh;
        }
    }
}
