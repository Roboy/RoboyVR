using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControllerManager : Singleton<ControllerManager> {

    public SteamVR_TrackedObject Controller_left;
    public SteamVR_TrackedObject Controller_right;
    

    public List<ControllerTool> Left_hand = new List<ControllerTool>();
    public List<ControllerTool> Right_hand = new List<ControllerTool>();

    [SerializeField]
    bool m_controller_left_set = false;
    [SerializeField]
    bool m_controller_right_set = false;

    SteamVR_Controller.Device m_SteamVRDevice_left;
    SteamVR_Controller.Device m_SteamVRDevice_right;

    IEnumerator checkForControllers()
    {
        
        while (!Controller_left.isValid)
            yield return null;

        m_SteamVRDevice_left = SteamVR_Controller.Input((int)Controller_left.index);

        while (!Controller_right.isValid)
            yield return null;

        m_SteamVRDevice_right = SteamVR_Controller.Input((int)Controller_right.index);


        while (!m_controller_left_set)
        {
            if (m_SteamVRDevice_left.GetHairTriggerDown())
            {

                foreach (ControllerTool t in Left_hand)
                {
                    Instantiate(t, Controller_left.transform);
                }

                m_controller_left_set = true;
            }
            yield return null;
        }

        while (!m_controller_right_set)
        {
            if (m_SteamVRDevice_right.GetHairTriggerDown())
            {

                foreach (ControllerTool t in Right_hand)
                {
                    Instantiate(t, Controller_right.transform);
                }

                m_controller_right_set = true;
            }
            yield return null;
        }

    }
    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

	
	}
}
