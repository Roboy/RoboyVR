using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;

public class BeRoboyManager : MonoBehaviour {

    /// <summary>
    /// The HMD main camera
    /// </summary>
    [SerializeField]
    private GameObject m_Cam;

    /// <summary>
    /// Is the main camera initialized or not
    /// </summary>
    private bool m_CamInitialized = false;

    /// <summary>
    /// Variable to determine if headset was rotated
    /// </summary>
    private float current_Angle = 0.0f;

    // Use this for initialization
    void Start () {

        //Looking for the HMD camera in scene
        if (!m_CamInitialized)
            m_Cam = GameObject.FindGameObjectWithTag("MainCamera");
        if (m_Cam != null)
            m_CamInitialized = true;
        else
            Debug.Log("No Camera found!");

    }
	
	// Update is called once per frame
	void Update () {

        //Looking for the HMD camera in scene
        if (!m_CamInitialized)
        {
            m_Cam = GameObject.FindGameObjectWithTag("MainCamera");
            if (m_Cam != null)
                m_CamInitialized = true;
            else
                Debug.Log("No Camera found!");
        }
        else
        {
            //Check whether the user has rotated the headset or not
            if (current_Angle != m_Cam.transform.eulerAngles.y)
            {
                //If the headset was rotated, rotate roboy
                transform.RotateAround(m_Cam.transform.localPosition, Vector3.up, m_Cam.transform.eulerAngles.y - current_Angle);
            }
            current_Angle = m_Cam.transform.eulerAngles.y;

            //Move roboy accordingly to headset movement
            Quaternion headRotation = InputTracking.GetLocalRotation(VRNode.Head);
            transform.position = m_Cam.transform.position + (headRotation * Vector3.forward) * (-0.3f);
        }
    }
}
