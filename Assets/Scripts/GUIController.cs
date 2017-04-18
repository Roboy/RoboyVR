using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;
using VRTK.Examples.Utilities;

/// <summary>
/// GUIController is attached on another controller as the Tools like ShootingTool or SelectorTool.
/// It is mainly responsible for animating so the following tasks refer always to animation:
/// - manage the switch between selection mode and panel mode
/// - manage switch between different panel modes
/// - manage page switch inside a panel mode
/// - <b>NOTICE: Right now GUIController is not inheriting from ControllerTool as we implemented this script at the beginning of the project.
/// This will be changed soon, so be aware that this documentation could be out of date!</b>
/// </summary>
public class GUIController : MonoBehaviour {

    /// <summary>
    /// Public variable for outside classes to track input.
    /// </summary>
    public SteamVR_Controller.Device Controller { get { return m_SteamVRDevice; } }

    /// <summary>
    /// Public variable for outside classes to track controller events.
    /// </summary>
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }

    /// <summary>
    /// Property which holds a dictionary to store a reference to the standard position of panels in panel mode.
    /// </summary>
    public Dictionary<UIPanelAlignment, FadePanelStruct> UIFadePanels {
        get { return m_UIFadePanels; }
    }

    /// <summary>
    /// Private variable to track controller input.
    /// </summary>
    private SteamVR_Controller.Device m_SteamVRDevice;

    /// <summary>
    /// Private variable to track controller identity.
    /// </summary>
    private SteamVR_TrackedObject m_SteamVRController;

    /// <summary>
    /// Private variable to track controller events.
    /// </summary>
    private SteamVR_TrackedController m_SteamVRTrackedController;

    /// <summary>
    /// Dictionary to store a reference to all UI Panels which are created at the start of the scene.
    /// </summary>
    private Dictionary<RoboyPart, UIPanelRoboyPart> m_RoboyPartPanelsDic =
    new Dictionary<RoboyPart, UIPanelRoboyPart>();

    /// <summary>
    /// Prefab variable for a roboy UI panel.
    /// </summary>
    public UIPanelRoboyPart UIPanelRoboyPartPrefab;

    /// <summary>
    /// Struct to store the position where a panel should fade in and out.
    /// </summary>
    public struct FadePanelStruct
    {
        public Transform FadeInPanel;
        public Transform FadeOutPanel;
        public Transform FadeStandardPanel;
    }

    /// <summary>
    /// Enum for possible panel alignments.
    /// </summary>
    public enum UIPanelAlignment
    {
        Left,
        Top,
        Right
    }

    /// <summary>
    /// Dictionary to store a reference to the standard position of panels in panel mode.
    /// </summary>
    private Dictionary<UIPanelAlignment, FadePanelStruct> m_UIFadePanels = new Dictionary<UIPanelAlignment, FadePanelStruct>();

    /// <summary>
    /// Reference to the SelectionPanel.
    /// </summary>
    private SelectionPanel m_SelectionPanel;

    /// <summary>
    /// Initializes the controller variables.
    /// Intializes the UI Panels and creates them for every roboy part for every panel mode.
    /// </summary>
    void Start () {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();
        // Find the controller and initialize the values to default
        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRDevice.Update();

        m_SteamVRTrackedController = GetComponentInParent<SteamVR_TrackedController>();

        // Find all template panels for fading
        List<Transform> allFadePanels =
            gameObject.GetComponentsInChildren<Transform>().Where(panel => panel.tag.Equals("FadePanelStruct")).ToList();

        // Initialize the fade panels
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

        // get the selection panel
        m_SelectionPanel = GetComponentInChildren<SelectionPanel>();

        // initialize the UI panels
        InitializePanels();
    }

    /// <summary>
    /// Checks the touchpad input of the controller and acts accordingly:
    /// -# Left: changes to previous panel if in panel mode
    /// -# Right: changes to next panel if in panel mode
    /// -# Top: changes between GUI modes
    /// -# Bottom: changes the page of the current panel if in panel mode
    /// </summary>
    /// <param name="touchpadStatus"></param>
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
    }

    /// <summary>
    /// Initialize the position of all panels and set their corresponding roboy part reference.
    /// </summary>
    public void InitializePanels()
    {
        foreach (var roboyPart in RoboyManager.Instance.RoboyParts)
        {
            // instantiate a copy of the prefab
            UIPanelRoboyPart uiPanelRoboyPart = Instantiate(UIPanelRoboyPartPrefab, Vector3.zero, Quaternion.identity);
            uiPanelRoboyPart.RoboyPart = roboyPart.Value;

            // initialize the panel modes
            uiPanelRoboyPart.InitializePanelModes(roboyPart.Value.MotorCount);
            // set the transform values
            uiPanelRoboyPart.transform.SetParent(transform, false);
            uiPanelRoboyPart.transform.localPosition = Vector3.zero;
            uiPanelRoboyPart.transform.localRotation = Quaternion.identity;
            uiPanelRoboyPart.transform.localScale = Vector3.one;
            uiPanelRoboyPart.gameObject.SetActive(false);
            // save the ui panel in a dictionary with roboypart component as index
            m_RoboyPartPanelsDic.Add(roboyPart.Value, uiPanelRoboyPart);
        }
    }

    /// <summary>
    /// Changes the page of the current panel if the current GUI mode is set to panel mode.
    /// </summary>
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

    /// <summary>
    /// Changes to the next panel if the current GUI mode if set to panel mode.
    /// </summary>
    private void changepanelsToNextMode()
    {
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            // get the selected roboy parts
            List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
            // get their roboy part components
            List<RoboyPart> selectedRoboyParts =
                selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();
            // start the animation to the next panel
            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].ChangeToNextMode();
            }
            // change the panel mode in the modemanager
            ModeManager.Instance.ChangePanelModeNext();
            // update the current text for the panel mode
            m_SelectionPanel.CurrentPanelModeText.text = ModeManager.Instance.CurrentPanelmode.ToString().Replace("_", " ");
        }
    }

    /// <summary>
    /// Changes to the previous panel if the current GUI mode if set to panel mode.
    /// </summary>
    private void changeToPreviousMode()
    {
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            // get the selected roboy parts 
            List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
            // get their roboy part components
            List<RoboyPart> selectedRoboyParts =
                selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();
            // start the animation to the previous panel
            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].ChangeToPreviousMode();
            }
            // change the panel mode in the modemanager
            ModeManager.Instance.ChangePanelModePrevious();
            // update the current text for the panel mode
            m_SelectionPanel.CurrentPanelModeText.text = ModeManager.Instance.CurrentPanelmode.ToString().Replace("_", " ");
        }
    }
    /// <summary>
    /// Changes GUI mode between selection and panel mode.
    /// </summary>
    /// <returns></returns>
    private IEnumerator changeGUIMode()
    {
        // get the selectet parts
        List<SelectableObject> selectedRoboyPartsOBJ = SelectorManager.Instance.SelectedParts;
        // get the roboypart components
        List<RoboyPart> selectedRoboyParts =
            selectedRoboyPartsOBJ.Select(roboyPart => roboyPart.GetComponent<RoboyPart>()).ToList();
        // if none found, return
        if (selectedRoboyParts.Count == 0)
            yield break;
        // else check current GUI mode
        if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.Selection)
        {
            // start shrink animation
            yield return StartCoroutine(m_SelectionPanel.shrinkCoroutine());
            positionPanels();

            // fade in the panels
            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].gameObject.SetActive(true);
                m_RoboyPartPanelsDic[roboyPart].FadeIn();
            }

            // set the current panel mode to MotorForce
            ModeManager.Instance.ResetPanelMode();

            // update panel mode text
            m_SelectionPanel.CurrentPanelModeText.gameObject.SetActive(true);

            m_SelectionPanel.CurrentPanelModeText.text = ModeManager.Instance.CurrentPanelmode.ToString().Replace("_", " ");
        }
        else if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIPanels)
        {
            // fade out the ui panels
            foreach (var roboyPart in selectedRoboyParts)
            {
                m_RoboyPartPanelsDic[roboyPart].FadeOut();
            }
            // wait for the animation
            yield return new WaitForSeconds(0.2f);
            // set the text to inactive
            m_SelectionPanel.CurrentPanelModeText.gameObject.SetActive(false);
            // start enlarge animation
            m_SelectionPanel.Enlarge();
        }
        // update GUI mode in modemanager
        ModeManager.Instance.ChangeGUIMode();
    }


    /// <summary>
    /// Positions the panels according to the template panel positions in the editor.
    /// </summary>
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
