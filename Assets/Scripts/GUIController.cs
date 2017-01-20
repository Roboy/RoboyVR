using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GUIController : MonoBehaviour {

    public SteamVR_Controller.Device SteamController { get { return m_SteamVRDevice; } }

    // Use this for initialization
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;

    void Start () {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRDevice.Update();
    }
	
	// Update is called once per frame
	void Update () {
		
	}

}
