using System.Collections;
using System.Collections.Generic;
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
        m_CurrentPanelIndex = 0;
        m_UIPanels[m_CurrentPanelIndex].gameObject.SetActive(true);
    }

    public IEnumerator ChangePanel()
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

}
