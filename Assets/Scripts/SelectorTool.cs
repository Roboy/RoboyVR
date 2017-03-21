using System.Collections;
using UnityEngine; 

[RequireComponent(typeof(LineRenderer))]
public class SelectorTool : ControllerTool {


    private LineRenderer m_LineRenderer;
    private SelectableObject m_LastSelectedObject;
    private float m_RayDistance = 3f;

    void Start()
    {
        m_LineRenderer = GetComponent<LineRenderer>();         
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
                        Vibrate();
                    }
                    else
                    {
                        hittedObject.SetStateTargeted();
                    }

                    Debug.Log("SelectorToolID: " + m_SteamVRDevice.index);

                    if (m_SteamVRDevice.GetHairTriggerDown())
                        {
                            hittedObject.SetStateSelected();
                            //Vibrate();
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
