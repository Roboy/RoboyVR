using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;
using VRTK.Examples.Utilities;

public class GUIController : MonoBehaviour {

    public SteamVR_Controller.Device Controller { get { return m_SteamVRDevice; } }
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }
    public Dictionary<int, UIPanel> UIPanels { get { return m_UIPanels; } }
    [Range(0.1f, 2f)] public float ChangeDuration;

    // Use this for initialization
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    private SteamVR_TrackedController m_SteamVRTrackedController;

    private Dictionary<int, UIPanel> m_UIPanels = new Dictionary<int, UIPanel>();
    private int m_CurrentPanelIndex = -1;

    private Transform m_FadeInPanel;
    private Transform m_FadeOutPanel;
    private Transform m_FadeStandardPanel;

    private struct FadePanelStruct
    {
        public Transform FadeInPanel;
        public Transform FadeOutPanel;
        public Transform FadeStandardPanel;
    }

    private Dictionary<string, FadePanelStruct> m_FadePanels = new Dictionary<string, FadePanelStruct>();

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
            else
            {
                switch (t.tag)
                {
                    case "FadeInPanel":
                        m_FadeInPanel = t;
                        t.gameObject.SetActive(false);
                        break;
                    case "FadeOutPanel":
                        m_FadeOutPanel = t;
                        t.gameObject.SetActive(false);
                        break;
                    case "FadeStandardPanel":
                        m_FadeStandardPanel = t;
                        t.gameObject.SetActive(false);
                        break;
                }
            }
        }

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

        Debug.Log(m_FadePanels.Count);

        m_CurrentPanelIndex = 0;
        m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(true);
    }

    public void ChangePanel()
    {
        StartCoroutine(changePanelCoroutine());
    }

    private IEnumerator changePanelCoroutine()
    {
        if (InputManager.Instance.GUIController_TouchpadStatus == InputManager.TouchpadStatus.Right)
        {
            yield return StartCoroutine(changePanelRight(ChangeDuration));
        }
        else if (InputManager.Instance.GUIController_TouchpadStatus == InputManager.TouchpadStatus.Left)
        {
            yield return StartCoroutine(changePanelLeft(ChangeDuration));
        }
        yield return null;
    }

    public IEnumerator ExpandPanel()
    {
        yield return StartCoroutine(expandPanelCoroutine());
    }

    private IEnumerator changePanelRight(float duration)
    {
        int nextPanelIndex = (m_CurrentPanelIndex + 1) % m_UIPanels.Count;
        m_UIPanels[nextPanelIndex].gameObject.SetActive(true);

        float currDuration = 0;
        CanvasGroup cgFadeIn = m_UIPanels[nextPanelIndex].GetComponent<CanvasGroup>();
        CanvasGroup cgFadeOut = m_UIPanels[m_CurrentPanelIndex].GetComponent<CanvasGroup>();

        if (cgFadeOut == null || cgFadeIn == null)
            yield break;

        cgFadeOut.alpha = 1f;
        cgFadeIn.alpha = 0f;

        while (currDuration < duration)
        {
            m_UIPanels[m_CurrentPanelIndex].transform.localPosition = Vector3.Slerp(m_FadeStandardPanel.localPosition,
                m_FadeOutPanel.localPosition, currDuration/duration);

            m_UIPanels[nextPanelIndex].transform.localPosition = Vector3.Slerp(m_FadeInPanel.localPosition,
                m_FadeStandardPanel.localPosition, currDuration / duration);

            cgFadeOut.alpha = Mathf.Lerp(1f, 0f, currDuration/duration);
            cgFadeIn.alpha = Mathf.Lerp(0f, 1f, currDuration/duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(false);
        m_CurrentPanelIndex = nextPanelIndex;
    }

    private IEnumerator changePanelLeft(float duration)
    {
        int nextPanelIndex;
        if (m_CurrentPanelIndex == 0)
            nextPanelIndex = m_UIPanels.Count - 1;
        else
        {
            nextPanelIndex = m_CurrentPanelIndex - 1;
        }
        m_UIPanels[nextPanelIndex].gameObject.SetActive(true);
        float currDuration = 0;
        CanvasGroup cgFadeIn = m_UIPanels[nextPanelIndex].GetComponent<CanvasGroup>();
        CanvasGroup cgFadeOut = m_UIPanels[m_CurrentPanelIndex].GetComponent<CanvasGroup>();

        if (cgFadeOut == null || cgFadeIn == null)
            yield break;

        cgFadeOut.alpha = 1f;
        cgFadeIn.alpha = 0f;

        while (currDuration < duration)
        {
            m_UIPanels[m_CurrentPanelIndex].transform.localPosition = Vector3.Slerp(m_FadeStandardPanel.localPosition,
                m_FadeInPanel.localPosition, currDuration / duration);

            m_UIPanels[nextPanelIndex].transform.localPosition = Vector3.Slerp(m_FadeOutPanel.localPosition,
                m_FadeStandardPanel.localPosition, currDuration / duration);

            cgFadeOut.alpha = Mathf.Lerp(1f, 0f, currDuration / duration);
            cgFadeIn.alpha = Mathf.Lerp(0f, 1f, currDuration / duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(false);
        m_CurrentPanelIndex = nextPanelIndex;
    }

    //private IEnumerator showDetailedPanels()
    //{

    //}

    private IEnumerator expandPanelCoroutine()
    {
        // Imagine code for shrink middle panel

        // UGLY CODE UGLY CODE CIC SUCKS DICKS

        int panelsSelected = SelectorManager.Instance.SelectedParts.Count;

        if (ModeManager.Instance.CurrentViewmode == ModeManager.Viewmode.Comparison &&
            ModeManager.Instance.CurrentPanelmode == ModeManager.Panelmode.Selection)
        {
            if (panelsSelected == 1)
            {
                GameObject topPanel = new GameObject("TopPanel");
                topPanel.transform.parent = transform;
                topPanel.transform.position = m_FadePanels["Top"].FadeStandardPanel.position;
                topPanel.transform.rotation = m_FadePanels["Top"].FadeStandardPanel.rotation;
                topPanel.transform.localScale = Vector3.zero;
                UIPanel uiComp = topPanel.AddComponent<UIPanel>();
                uiComp.Alignment = UIPanel.Side.Top;
            }
            else if (panelsSelected == 2)
            {
                GameObject leftPanel = new GameObject("LeftPanel");
                leftPanel.transform.parent = transform;
                leftPanel.transform.position = m_FadePanels["Left"].FadeStandardPanel.position;
                leftPanel.transform.rotation = m_FadePanels["Left"].FadeStandardPanel.rotation;
                leftPanel.transform.localScale = Vector3.zero;
                UIPanel uiCompL = leftPanel.AddComponent<UIPanel>();
                uiCompL.Alignment = UIPanel.Side.Left;

                GameObject rightPanel = new GameObject("RightPanel");
                rightPanel.transform.parent = transform;
                rightPanel.transform.position = m_FadePanels["Right"].FadeStandardPanel.position;
                rightPanel.transform.rotation = m_FadePanels["Right"].FadeStandardPanel.rotation;
                rightPanel.transform.localScale = Vector3.zero;
                UIPanel uiCompR = rightPanel.AddComponent<UIPanel>();
                uiCompR.Alignment = UIPanel.Side.Right;
            }
            else if (panelsSelected == 3)
            {
                GameObject topPanel = new GameObject("TopPanel");
                topPanel.transform.parent = transform;
                topPanel.transform.position = m_FadePanels["Top"].FadeStandardPanel.position;
                topPanel.transform.rotation = m_FadePanels["Top"].FadeStandardPanel.rotation;
                topPanel.transform.localScale = Vector3.zero;
                UIPanel uiComp = topPanel.AddComponent<UIPanel>();
                uiComp.Alignment = UIPanel.Side.Top;

                GameObject leftPanel = new GameObject("LeftPanel");
                leftPanel.transform.parent = transform;
                leftPanel.transform.position = m_FadePanels["Left"].FadeStandardPanel.position;
                leftPanel.transform.rotation = m_FadePanels["Left"].FadeStandardPanel.rotation;
                leftPanel.transform.localScale = Vector3.zero;
                UIPanel uiCompL = leftPanel.AddComponent<UIPanel>();
                uiCompL.Alignment = UIPanel.Side.Left;

                GameObject rightPanel = new GameObject("RightPanel");
                rightPanel.transform.parent = transform;
                rightPanel.transform.position = m_FadePanels["Right"].FadeStandardPanel.position;
                rightPanel.transform.rotation = m_FadePanels["Right"].FadeStandardPanel.rotation;
                rightPanel.transform.localScale = Vector3.zero;
                UIPanel uiCompR = rightPanel.AddComponent<UIPanel>();
                uiCompR.Alignment = UIPanel.Side.Right;
            }
        }
        yield return null;
    }


}
