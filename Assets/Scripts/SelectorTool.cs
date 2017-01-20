using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using Valve.VR;

[RequireComponent(typeof(LineRenderer))]
public class SelectorTool : MonoBehaviour {

    public SteamVR_Controller.Device SteamController { get { return m_SteamVRDevice; }  }

    private LineRenderer m_LineRenderer;
    private SelectableObject m_LastSelectedObject;
    private float m_RayDistance = 3f;

    // Update is called once per frame
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    

    void Start()
    {
        m_LineRenderer = GetComponent<LineRenderer>();

        // Find Selector tool and the corresponding controller
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRDevice.Update();
    }

    public void GetRayFromController()
    {
        RaycastHit hit;

        m_LineRenderer.SetPosition(0, transform.position);

        if (Physics.Raycast(transform.position, transform.forward, out hit, m_RayDistance))
        {
            m_LineRenderer.SetPosition(1, hit.point);

            SelectableObject hittedObject;

            if (hit.transform.tag.Equals("RoboyUI"))
            {
                hittedObject = RoboyManager.Instance.RoboyParts[hit.transform.name].GetComponent<SelectableObject>();
            }
            else
            {
                hittedObject = hit.transform.gameObject.GetComponent<SelectableObject>();
            }
                if (hittedObject)
                {
                    if (m_LastSelectedObject != hittedObject)
                    {
                        if (m_LastSelectedObject != null)
                            m_LastSelectedObject.SetStateDefault();

                        m_LastSelectedObject = hittedObject;
                    }


                    hittedObject.SetStateTargeted();

                    if (m_SteamVRDevice.GetHairTriggerDown())
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
    }
}
