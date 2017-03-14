using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;
using VRTK.Examples.Utilities;

public class GUIController : MonoBehaviour {

    public SteamVR_Controller.Device Controller { get { return m_SteamVRDevice; } }
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }

    public Dictionary<UIPanelAlignment, FadePanelStruct> UIFadePanels {
        get { return m_UIFadePanels; }
    }

    public enum UIPanelAlignment
    {
        Left,
        Top,
        Right
    }

    // Use this for initialization
    private SteamVR_Controller.Device m_SteamVRDevice;
    private SteamVR_TrackedObject m_SteamVRController;
    private SteamVR_TrackedController m_SteamVRTrackedController;

    private Dictionary<RoboyPart, UIPanelRoboyPart> m_RoboyPartPanelsDic =
    new Dictionary<RoboyPart, UIPanelRoboyPart>();

    public UIPanelRoboyPart UIPanelRoboyPartPrefab;

    public struct FadePanelStruct
    {
        public Transform FadeInPanel;
        public Transform FadeOutPanel;
        public Transform FadeStandardPanel;
    }

    private Dictionary<UIPanelAlignment, FadePanelStruct> m_UIFadePanels = new Dictionary<UIPanelAlignment, FadePanelStruct>();

    private SelectionPanel m_SelectionPanel;

    void Start () {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
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

            m_UIFadePanels.Add(fadeInPanel.parent.GetComponent<FadePanelTemplate>().Alignment, fadePanelStruct);

            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeInPanel").gameObject.SetActive(false);
            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeOutPanel").gameObject.SetActive(false);
            fadePanel.gameObject.GetComponentInChildWithTag<Transform>("FadeStandardPanel").gameObject.SetActive(false);
        }

        m_SelectionPanel = GetComponentInChildren<SelectionPanel>();

        InitializePanels();
    }

    public void CheckTouchPad(InputManager.TouchpadStatus touchpadStatus)
    {
        switch (touchpadStatus)
        {
            case InputManager.TouchpadStatus.Left:
                changeToPreviousMode();
                break;
            case InputManager.TouchpadStatus.Right:
                changepanelsToNextMode();
                break;
            case InputManager.TouchpadStatus.Top:
                StartCoroutine(changeGUIMode());
                break;
            case InputManager.TouchpadStatus.Bottom:
                changePageOfPanel();
                break;
        }

        Debug.Log("Checktouchpad");
    }

    public void InitializePanels()
    {
        foreach (var roboyPart in RoboyManager.Instance.RoboyParts)
        {

            UIPanelRoboyPart uiPanelRoboyPart = Instantiate(UIPanelRoboyPartPrefab, Vector3.zero, Quaternion.identity);
            uiPanelRoboyPart.RoboyPart = roboyPart.Value;

            
            uiPanelRoboyPart.InitializePanelModes(roboyPart.Value.MotorCount);
            uiPanelRoboyPart.transform.SetParent(transform, false);
            uiPanelRoboyPart.transform.localPosition = Vector3.zero;
            uiPanelRoboyPart.transform.localRotation = Quaternion.identity;
            uiPanelRoboyPart.transform.localScale = Vector3.one;
            uiPanelRoboyPart.gameObject.SetActive(false);

            m_RoboyPartPanelsDic.Add(roboyPart.Value, uiPanelRoboyPart);
        }
    }

    private void changePageOfPanel()
    {
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
            List<RoboyPart> selectedRoboyParts =
                selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();

            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].ChangePage();
            }
        }
    }

    private void changepanelsToNextMode()
    {
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
            List<RoboyPart> selectedRoboyParts =
                selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();

            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].ChangeToNextMode();
            }
        }
    }

    private void changeToPreviousMode()
    {
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
            List<RoboyPart> selectedRoboyParts =
                selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();

            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].ChangeToPreviousMode();
            }
        }
    }

    private IEnumerator changeGUIMode()
    {
        List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
        List<RoboyPart> selectedRoboyParts =
            selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();

        if (selectedRoboyParts.Count == 0)
            yield break;

        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.Selection)
        {
            yield return m_SelectionPanel.shrinkCoroutine();
            positionPanels();

            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].gameObject.SetActive(true);
                m_RoboyPartPanelsDic[roboyPart].FadeIn();
            }
        }
        else if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].FadeOut();
            }

            yield return new WaitForSeconds(0.2f);

            m_SelectionPanel.Enlarge();
        }
        Debug.Log("changeGUIMode");
        ModeManager.Instance.ChangeGUIMode();
    }



    private void positionPanels()
    {
        int panelsSelected = SelectorManager.Instance.SelectedParts.Count;

        List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
        List<RoboyPart> selectedRoboyParts =
            selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();

        if (panelsSelected == 1)
        {
            m_RoboyPartPanelsDic[selectedRoboyParts[0]].SetPosition(UIPanelAlignment.Top);
        }
        else if (panelsSelected == 2)
        {
            m_RoboyPartPanelsDic[selectedRoboyParts[0]].SetPosition(UIPanelAlignment.Left);
            m_RoboyPartPanelsDic[selectedRoboyParts[1]].SetPosition(UIPanelAlignment.Right);
        }
        else if (panelsSelected == 3)
        {
            m_RoboyPartPanelsDic[selectedRoboyParts[0]].SetPosition(UIPanelAlignment.Left);
            m_RoboyPartPanelsDic[selectedRoboyParts[1]].SetPosition(UIPanelAlignment.Top);
            m_RoboyPartPanelsDic[selectedRoboyParts[2]].SetPosition(UIPanelAlignment.Right);
        }
    }
}
