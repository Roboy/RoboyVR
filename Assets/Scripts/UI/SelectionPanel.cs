using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// SelectionPanel is the panel where you can select roboy parts with the SelectorTool on a GUI interface.
/// Whereas the components inside the panel provide functions to switch between selection states, this class is responsible to animate the switch between Selection Mode and GUI Panel mode.
/// </summary>
public class SelectionPanel : MonoBehaviour
{
    /// <summary>
    /// Reference to the text component to display the current panel mode like MotorForce etc.
    /// </summary>
    public Text CurrentPanelModeText;

    /// <summary>
    /// Private RectTransform component for animation purposes.
    /// </summary>
    private RectTransform m_RectTransform;

    /// <summary>
    /// List of all canvas groups to change the alpha value.
    /// </summary>
    private List<CanvasGroup> m_ChildCanvasGroups = new List<CanvasGroup>();
    /// <summary>
    /// List of all colliders on the UI elements to switch them off and on.
    /// </summary>
    private List<BoxCollider> m_ChildBoxColliders = new List<BoxCollider>();

    /// <summary>
    /// Initializes all variables like the RectTransform and the lists.
    /// </summary>
    void Awake()
    {
        m_RectTransform = GetComponent<RectTransform>();

        foreach (Transform t in transform)
        {
            CanvasGroup cG;

            // find all canvas groups
            if ((cG = t.GetComponent<CanvasGroup>()) != null)
            {
                m_ChildCanvasGroups.Add(cG);
            }

            BoxCollider bC;
            // find all colliders
            if ((bC = t.GetComponent<BoxCollider>()) != null)
            {
                m_ChildBoxColliders.Add(bC);
            }
        }
        // turn of the text as the default gui mode is selection mode
        CurrentPanelModeText.gameObject.SetActive(false);
    }

    private void Start()
    {
        initializeUI();
    }

    /// <summary>
    /// Starts a coroutine to shrink the selection panel.
    /// </summary>
    public void Shrink()
    {
        StartCoroutine(shrinkCoroutine());   
    }

    /// <summary>
    /// Starts a coroutine to enlarge the selection panel.
    /// </summary>
    public void Enlarge()
    {
        StartCoroutine(enlargeCoroutine());
    }

    /// <summary>
    /// Coroutine to shrink the selection panel.
    /// Fades out the UI elements, turns off the colliders and shrinks the selection panel.
    /// </summary>
    /// <returns></returns>
    public IEnumerator shrinkCoroutine()
    {
        // Initialze values
        float startWidth = m_RectTransform.sizeDelta.x;
        float endWidth = 35f;

        float duration = 0.25f;
        float currentDuration = 0f;

        // turn off all colliders
        foreach (BoxCollider bC in m_ChildBoxColliders)
        {
            bC.enabled = false;
        }

        // while the animation is playing
        while (currentDuration < duration)
        {
            // fade out the ui elements
            foreach (var c in m_ChildCanvasGroups)
            {
                c.alpha = Mathf.Lerp(1f, 0f, currentDuration/duration);
            }
            currentDuration += Time.deltaTime;
            yield return null;
        }
        // set alpha values to 0
        foreach (var c in m_ChildCanvasGroups)
        {
            c.alpha = 0f;
        }

        currentDuration = 0f;

        // shrink the selection panel object itself
        while (currentDuration < duration)
        {
            m_RectTransform.sizeDelta = new Vector2(Mathf.Lerp(startWidth, endWidth, currentDuration/duration), m_RectTransform.sizeDelta.y);
            currentDuration += Time.deltaTime;
            yield return null;
        }
        m_RectTransform.sizeDelta = new Vector2(endWidth, m_RectTransform.sizeDelta.y);
    }

    /// <summary>
    /// Coroutine to enlarge the selection panel.
    /// Fades in the UI elements, turns on the colliders and enlarges the selection panel.
    /// </summary>
    /// <returns></returns>
    IEnumerator enlargeCoroutine()
    {
        float startWidth = m_RectTransform.sizeDelta.x;
        float endWidth = 170f;

        float duration = 0.25f;
        float currentDuration = 0f;

        while (currentDuration < duration)
        {
            m_RectTransform.sizeDelta = new Vector2(Mathf.Lerp(startWidth, endWidth, currentDuration / duration), m_RectTransform.sizeDelta.y);
            currentDuration += Time.deltaTime;
            yield return null;
        }
        m_RectTransform.sizeDelta = new Vector2(endWidth, m_RectTransform.sizeDelta.y);
        currentDuration = 0f;

        while (currentDuration < duration)
        {

            foreach (var c in m_ChildCanvasGroups)
            {
                c.alpha = Mathf.Lerp(0f, 1f, currentDuration / duration);
            }
            currentDuration += Time.deltaTime;
            yield return null;
        }

        foreach (var c in m_ChildCanvasGroups)
        {
            c.alpha = 1f;
        }
        
        foreach (BoxCollider bC in m_ChildBoxColliders)
        {
            bC.enabled = true;
        }       
    }

    private void initializeUI()
    {
        // This part does not always work! When The Left Controller is not active then the attached RoboyUI objects cannot be found as they are inactive as well
        // => UI dictionary is empty => selectableObject cannot find UI! ------------- MAYBE FIXED!!!!----------------------
        // Add an Outline component to all UI elements if they dont have one
        foreach (GameObject g in SelectorManager.Instance.UI_Elements.Values)
        {
            Outline outline;

            if (g.GetComponent<Outline>() == null)
                g.AddComponent<Outline>();
            outline = g.GetComponent<Outline>();

            outline.enabled = false;
            outline.effectDistance = new Vector2(0.2f, -0.2f);

            g.GetComponentInChildren<Text>().text = g.name;
        }
    }
}
