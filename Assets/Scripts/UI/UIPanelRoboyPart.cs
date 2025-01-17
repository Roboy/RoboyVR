﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.UI;

public class UIPanelRoboyPart : MonoBehaviour {

    public UIPanelPanelMode UIPanelModePrefab;

    public GUIController.UIPanelAlignment Alignment = GUIController.UIPanelAlignment.Left;

    public RoboyPart RoboyPart;

    public List<UIPanelPanelMode> UIPanelPanelModes = new List<UIPanelPanelMode>();

    public ModeManager.Panelmode CurrentPanelmode = ModeManager.Panelmode.Motor_Force;

    public Text NameText;

    public Text PageText;

    public int CurrentPageIndex;

    private int m_CurrentPanelIndex;

    public void InitializePanelModes(int motorCount)
    {
        if (motorCount < 1)
        {
            Debug.Log("Pages count cannot be smaller than 1!");
            return;
        }
        if (RoboyPart == null)
        {
            Debug.Log("Roboy part not assigned!");
            return;
        }

        int pagesCount = (RoboyPart.MotorCount + 3) / 4; //(motorCount + motorsPerPage - 1) / motorsPerPage

        PageText.text = 1 + "/" + pagesCount;

        foreach (var category in RoboyPart.Categories.Values)
        {
            UIPanelPanelMode panelMode = Instantiate(UIPanelModePrefab, Vector3.zero, Quaternion.identity);
            panelMode.PanelMode = category.Mode;
            panelMode.transform.SetParent(transform, false);
            panelMode.CreatePages(motorCount, pagesCount, RoboyPart);
            UIPanelPanelModes.Add(panelMode);

            panelMode.gameObject.SetActive(false);
        }

        m_CurrentPanelIndex = 0;

        UIPanelPanelModes[0].gameObject.SetActive(true);
    }

    public void ChangePage()
    {
        if (UIPanelPanelModes[m_CurrentPanelIndex].PanelPages.Count > 1)
        {
            int nextPage = (CurrentPageIndex + 1) % UIPanelPanelModes[m_CurrentPanelIndex].PanelPages.Count;
            UIPanelPanelModes[m_CurrentPanelIndex].PanelPages[CurrentPageIndex].gameObject.SetActive(false);
            UIPanelPanelModes[m_CurrentPanelIndex].PanelPages[nextPage].gameObject.SetActive(true);
            CurrentPageIndex = nextPage;

            PageText.text = CurrentPageIndex + 1 + "/" + UIPanelPanelModes[m_CurrentPanelIndex].PanelPages.Count;
        }
    }

    public void ChangeToNextMode()
    {
        StartCoroutine(changeToNextModeCoroutine());
    }

    public void ChangeToPreviousMode()
    {
        StartCoroutine(changeToPreviousModeCoroutine());
    }

    public void FadeIn()
    {
        NameText.text = RoboyPart.name;
        StartCoroutine(fadeInCoroutine());
    }

    public void FadeOut()
    {
        StartCoroutine(fadeOutCoroutine());
    }

    public void SetPosition(GUIController.UIPanelAlignment alignment)
    {
        Alignment = alignment;
        transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[alignment].FadeStandardPanel.transform.position;
    }

    private IEnumerator changeToNextModeCoroutine()
    {
        // Change to next panel mode
        CurrentPanelmode++;

        // Get the next panel index
        int nextPanelIndex = (m_CurrentPanelIndex + 1) % UIPanelPanelModes.Count;

        UIPanelPanelMode currentPanel = UIPanelPanelModes[m_CurrentPanelIndex];

        // Activate the next panel
        UIPanelPanelMode nextPanel = UIPanelPanelModes[nextPanelIndex];

        nextPanel.PanelPages[CurrentPageIndex].gameObject.SetActive(true);

        nextPanel.gameObject.SetActive(true);

        // Set start position of current panel
        currentPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.localPosition;

        // Set start position of next panel
        nextPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.localPosition;

        // Set alpha values of canvas groups
        currentPanel.CanvasGroup.alpha = 1f;
        nextPanel.CanvasGroup.alpha = 0f;

        // Lerp the alpha values and the position values
        float duration = 0.2f;
        float currDuration = 0f;

        while (currDuration < duration)
        {
            // Lerp the position of the current panel from standard panel to fade out panel
            currentPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.position,
                    currDuration/duration);

            // Lerp the next panel position from in to standard
            nextPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    currDuration/duration);

            // Lerp the alpha values of both canvas groups
            currentPanel.CanvasGroup.alpha = Mathf.Lerp(1f, 0f, currDuration/duration);
            nextPanel.CanvasGroup.alpha = Mathf.Lerp(0f, 1f, currDuration/duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        // Set alpha values and positions to final values
        currentPanel.CanvasGroup.alpha = 0f;
        nextPanel.CanvasGroup.alpha = 1f;

        currentPanel.transform.position = InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.position;
        nextPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position;

        // Deactivate the old panel and the current page of the old panel
        currentPanel.PanelPages[CurrentPageIndex].gameObject.SetActive(false);
        currentPanel.gameObject.SetActive(false);
        m_CurrentPanelIndex = nextPanelIndex;


    }

    private IEnumerator changeToPreviousModeCoroutine()
    {
        // Change to next panel mode
        CurrentPanelmode--;

        // Get the next panel index
        int nextPanelIndex;
        if(m_CurrentPanelIndex == 0)
            nextPanelIndex = UIPanelPanelModes.Count - 1;
        else
            nextPanelIndex = m_CurrentPanelIndex - 1;

        UIPanelPanelMode currentPanel = UIPanelPanelModes[m_CurrentPanelIndex];

        // Activate the next panel
        UIPanelPanelMode nextPanel = UIPanelPanelModes[nextPanelIndex];

        nextPanel.PanelPages[CurrentPageIndex].gameObject.SetActive(true);

        nextPanel.gameObject.SetActive(true);

        // Set start position of current panel
        currentPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.localPosition;

        // Set start position of next panel
        nextPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.localPosition;

        // Set alpha values of canvas groups
        currentPanel.CanvasGroup.alpha = 1f;
        nextPanel.CanvasGroup.alpha = 0f;

        // Lerp the alpha values and the position values
        float duration = 0.2f;
        float currDuration = 0f;

        while (currDuration < duration)
        {
            // Lerp the position of the current panel from standard panel to fade out panel
            currentPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.position,
                    currDuration / duration);

            // Lerp the next panel position from in to standard
            nextPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    currDuration / duration);

            // Lerp the alpha values of both canvas groups
            currentPanel.CanvasGroup.alpha = Mathf.Lerp(1f, 0f, currDuration / duration);
            nextPanel.CanvasGroup.alpha = Mathf.Lerp(0f, 1f, currDuration / duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        // Set alpha values and positions to final values
        currentPanel.CanvasGroup.alpha = 0f;
        nextPanel.CanvasGroup.alpha = 1f;

        currentPanel.transform.position = InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.position;
        nextPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position;

        // Deactivate the old panel and the current page of the old panel
        currentPanel.PanelPages[CurrentPageIndex].gameObject.SetActive(false);
        currentPanel.gameObject.SetActive(false);
        m_CurrentPanelIndex = nextPanelIndex;
    }

    private IEnumerator fadeInCoroutine()
    {
        // Reset index and mode
        m_CurrentPanelIndex = 0;
        CurrentPageIndex = 0;
        CurrentPanelmode = ModeManager.Panelmode.Motor_Force;
        PageText.text = 1 + "/" + UIPanelPanelModes[0].PanelPages.Count;

        UIPanelPanelMode currentPanel = UIPanelPanelModes[0];

        // Activate the first panel
        currentPanel.PanelPages[0].gameObject.SetActive(true);
        currentPanel.gameObject.SetActive(true);

        // Set start position of current panel
        currentPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.localPosition;

        // Set alpha values of canvas groups
        currentPanel.CanvasGroup.alpha = 0f;

        // Lerp the alpha values and the position values
        float duration = 0.2f;
        float currDuration = 0f;

        while (currDuration < duration)
        {
            // Lerp the position of the current panel from standard panel to fade out panel
            currentPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeInPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    currDuration / duration);

            // Lerp the alpha values of both canvas groups
            currentPanel.CanvasGroup.alpha = Mathf.Lerp(0f, 1f, currDuration / duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        // Set alpha values and positions to final values
        currentPanel.CanvasGroup.alpha = 1f;

        currentPanel.transform.position = InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position;
    }

    private IEnumerator fadeOutCoroutine()
    {
        UIPanelPanelMode currentPanel = UIPanelPanelModes[m_CurrentPanelIndex];

        // Set start position of current panel
        currentPanel.transform.position =
            InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.localPosition;

        // Set alpha values of canvas groups
        currentPanel.CanvasGroup.alpha = 1f;

        // Lerp the alpha values and the position values
        float duration = 0.2f;
        float currDuration = 0f;

        while (currDuration < duration)
        {
            // Lerp the position of the current panel from standard panel to fade out panel
            currentPanel.transform.position =
                Vector3.Slerp(
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeStandardPanel.position,
                    InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.position,
                    currDuration / duration);

            // Lerp the alpha values of both canvas groups
            currentPanel.CanvasGroup.alpha = Mathf.Lerp(1f, 0f, currDuration / duration);

            currDuration += Time.deltaTime;
            yield return null;
        }

        // Set alpha values and positions to final values
        currentPanel.CanvasGroup.alpha = 0f;

        currentPanel.transform.position = InputManager.Instance.GUI_Controller.UIFadePanels[Alignment].FadeOutPanel.position;

        // Deactivate the last current panel and the current page of the current mode
        currentPanel.PanelPages[CurrentPageIndex].gameObject.SetActive(false);
        currentPanel.gameObject.SetActive(false);

        m_CurrentPanelIndex = 0;
        CurrentPageIndex = 0;

        gameObject.SetActive(false);        
    }
}
