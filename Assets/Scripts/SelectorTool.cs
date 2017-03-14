using System.Collections;
using UnityEngine; 

[RequireComponent(typeof(LineRenderer))]
public class SelectorTool : MonoBehaviour {

    public SteamVR_Controller.Device Controller { get { return m_SteamVRDevice; }  }
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }

    private LineRenderer m_LineRenderer;
    private SelectableObject m_LastSelectedObject;
    private float m_RayDistance = 3f;

    // Update is called once per frame
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    private SteamVR_TrackedController m_SteamVRTrackedController;
    

    void Start()
    {
        m_LineRenderer = GetComponent<LineRenderer>();

        // Find Selector tool and the corresponding controller
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRDevice.Update();

        m_SteamVRTrackedController = GetComponentInParent<SteamVR_TrackedController>();

          
    }

    public void GetRayFromController()
    {
        RaycastHit hit;
        m_LineRenderer.SetPosition(0, transform.position);

        if (Physics.Raycast(transform.position, transform.forward, out hit, m_RayDistance))
        {
            m_LineRenderer.SetPosition(1, hit.point);
            SelectableObject hittedObject;

            // CHANGE THIS IN FUTURE ONLY TEST
            if (ModeManager.Instance.CurrentGUIMode != ModeManager.GUIMode.Selection)
                return;

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
                        StartCoroutine(vibrateController());

                    }
                    else
                    {
                        hittedObject.SetStateTargeted();
                    }
                    

                    if (m_SteamVRDevice.GetHairTriggerDown())
                    {
                        hittedObject.SetStateSelected();
                        SteamVR_Controller.Input((int)m_SteamVRController.index).TriggerHapticPulse(500);
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

    private IEnumerator vibrateController()
    {
        float duration = 0.25f;
        float currDuration = 0f;
        float vibrationStrength = 250f;

        while (currDuration < duration)
        {
            float sinValue = Mathf.Sin(currDuration / duration * Mathf.PI) * vibrationStrength;
            SteamVR_Controller.Input((int)m_SteamVRController.index).TriggerHapticPulse((ushort)sinValue);
            currDuration += Time.fixedDeltaTime;
            yield return Time.fixedDeltaTime;
        }
    }
}
