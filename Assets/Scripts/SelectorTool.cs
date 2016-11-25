using UnityEngine;
using System.Collections;
using Valve.VR;

public class SelectorTool : MonoBehaviour {

	SteamVR_Controller.Device device;
	SteamVR_TrackedObject controller;


    private LineRenderer m_LineRenderer;

    private SelectableObject m_LastSelectedObject;

    private float m_RayDistance = 3f;

    // Use this for initialization
    void Start () {
		controller = GetComponentInParent<SteamVR_TrackedObject> ();
        m_LineRenderer = GetComponent<LineRenderer>();
    }
	
	// Update is called once per frame
	void Update () {
		device = SteamVR_Controller.Input ((int)controller.index);
	    RaycastHit hit;

        m_LineRenderer.SetPosition(0, transform.position);

	    if (Physics.Raycast(transform.position, transform.forward, out hit, m_RayDistance))
	    {
	        m_LineRenderer.SetPosition(1, hit.point);

	        SelectableObject hittedObject = hit.transform.gameObject.GetComponent<SelectableObject>();
	        if (hittedObject)
	        {
	            if (m_LastSelectedObject != hittedObject)
	            {
                    if(m_LastSelectedObject != null)
                        m_LastSelectedObject.SetStateDefault();

	                m_LastSelectedObject = hittedObject;
	            }
	           

                hittedObject.SetStateTargeted();

	            if (device.GetHairTriggerDown())
	            {
                    hittedObject.SetStateSelected();
	            }

	        }
	    }
	    else
	    {
            m_LineRenderer.SetPosition(1, transform.position + transform.forward * m_RayDistance);

            if (m_LastSelectedObject != null)
                m_LastSelectedObject.SetStateDefault();

	        m_LastSelectedObject = null;
	    }
        
        // TOUCHPAD : Debug.Log("Axis0: " + device.GetAxis(EVRButtonId.k_EButton_Axis0));
        // TRIGGER : Debug.Log("Axis1: " + device.GetAxis(EVRButtonId.k_EButton_Axis1));
    }
}
