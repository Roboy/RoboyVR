using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class GUIController2 : MonoBehaviour
{

    public SteamVR_Controller.Device Controller
    {
        get { return m_SteamVRDevice; }
    }

    public SteamVR_TrackedController ControllerEventListener
    {
        get { return m_SteamVRTrackedController; }
    }

    public UIPanelRoboyPart UIPanelRoboyPartPrefab;

    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    private SteamVR_TrackedController m_SteamVRTrackedController;

    private Dictionary<RoboyPart, UIPanelRoboyPart> m_Panels =
        new Dictionary<RoboyPart, UIPanelRoboyPart>();

    private struct FadePanelStruct
    {
        public Transform FadeInPanel;
        public Transform FadeOutPanel;
        public Transform FadeStandardPanel;
    }

    private Dictionary<string, FadePanelStruct> m_FadePanels = new Dictionary<string, FadePanelStruct>();

    void Start()
    {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int) m_SteamVRController.index);
        m_SteamVRDevice.Update();

        m_SteamVRTrackedController = GetComponentInParent<SteamVR_TrackedController>();

        List<Transform> allFadePanels =
            gameObject.GetComponentsInChildren<Transform>().Where(panel => panel.tag.Equals("FadePanelStruct")).ToList();

        foreach (var fadePanel in allFadePanels)
        {
            Transform fadeInPanel = fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeInPanel");
            Transform fadeOutPanel = fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeOutPanel");
            Transform fadeStandardPanel = fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeStandardPanel");

            FadePanelStruct fadePanelStruct;
            fadePanelStruct.FadeInPanel = fadeInPanel;
            fadePanelStruct.FadeOutPanel = fadeOutPanel;
            fadePanelStruct.FadeStandardPanel = fadeStandardPanel;

            m_FadePanels.Add(fadeInPanel.parent.name, fadePanelStruct);

            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeInPanel").gameObject.SetActive(false);
            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeOutPanel").gameObject.SetActive(false);
            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeStandardPanel").gameObject.SetActive(false);
        }

        InitializePanels();
    }

    public void InitializePanels()
    {
        foreach (var roboyPart in RoboyManager.Instance.RoboyParts)
        {

            UIPanelRoboyPart uiPanelRoboyPart = Instantiate(UIPanelRoboyPartPrefab, transform.position, transform.rotation);
            uiPanelRoboyPart.RoboyPart = roboyPart.Value;
            uiPanelRoboyPart.CreatePagesForEachMode(roboyPart.Value.MotorCount);
            uiPanelRoboyPart.transform.parent = transform;

            m_Panels.Add(roboyPart.Value, uiPanelRoboyPart);
        }
    }
}

