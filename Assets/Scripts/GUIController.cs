using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GUIController : MonoBehaviour {

    public SteamVR_Controller.Device Controller { get { return m_SteamVRDevice; } }
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }
    public Dictionary<int, UIPanel> UIPanels { get { return m_UIPanels; } }

    // Use this for initialization
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    private SteamVR_TrackedController m_SteamVRTrackedController;

    private Dictionary<int, UIPanel> m_UIPanels = new Dictionary<int, UIPanel>();
    private int m_CurrentPanelIndex = -1;

    void Start () {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRDevice.Update();

        m_SteamVRTrackedController = GetComponentInParent<SteamVR_TrackedController>();

        foreach (Transform t in transform)
        {
            UIPanel panelScript;
            if (t.tag.Equals("UIPanel") && (panelScript = t.GetComponent<UIPanel>()) != null)
            {
                m_UIPanels.Add(panelScript.Index, panelScript);
                panelScript.gameObject.SetActive(false);
            }
        }
        m_CurrentPanelIndex = 0;
        m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(true);
    }
	
	// Update is called once per frame
	void Update () {
		
	}

    public void ChangePanel()
    {
        if (InputManager.Instance.GUIController_TouchpadStatus == InputManager.TouchpadStatus.Right)
        {
            m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(false);
            m_CurrentPanelIndex = (m_CurrentPanelIndex + 1) % m_UIPanels.Count;
            m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(true);
        }
        else if (InputManager.Instance.GUIController_TouchpadStatus == InputManager.TouchpadStatus.Left)
        {
            m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(false);
            if (m_CurrentPanelIndex == 0)
                m_CurrentPanelIndex = m_UIPanels.Count - 1;
            else
                m_CurrentPanelIndex--;
            m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(true);
        }
    }

}
