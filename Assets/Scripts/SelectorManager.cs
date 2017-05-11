using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// SelectorManager is responsible to hold references of all selected roboy parts and the corresponding UI elements.
/// </summary>
public class SelectorManager : Singleton<SelectorManager>
{
    /// <summary>
    /// Property which returns a dictionary of all UI elements in the SelectionPanel.
    /// </summary>
    public Dictionary<string, GameObject> UI_Elements { get { return m_UI_Elements; } }
    /// <summary>
    /// Reference of all currently selected roboy parts.
    /// </summary>
    public List<SelectableObject> SelectedParts { get { return m_SelectedParts; } }

    /// <summary>
    /// Integer to switch between single mode selection and normal mode collection.
    /// </summary>
    public int MaximumSelectableObjects {
        get { return m_CurrentMaximumSelectedObjects; }
        set { m_CurrentMaximumSelectedObjects = (int)Mathf.Clamp(value, 0f, m_MaximumSelectableObjects); }
    }

    /// <summary>
    /// TEMPORARY VARIABLE TO CHECK HOW MANY UI ELEMENTS ARE INITIALIZED
    /// </summary>
    public int RoboyUIElementsCount = 13;

    /// <summary>
    /// Transform of roboy model.
    /// </summary>
    private Transform m_Roboy;

    /// <summary>
    /// List of SelectableObject components of all roboy parts.
    /// </summary>
    private List<SelectableObject> m_RoboyParts = new List<SelectableObject>();

    /// <summary>
    /// List of SelectableObject components of all selected parts.
    /// </summary>
    private List<SelectableObject> m_SelectedParts = new List<SelectableObject>();

    /// <summary>
    /// Maximum cound of selectable objects in multiple selection mode.
    /// </summary>
    private int m_MaximumSelectableObjects = 3;

    /// <summary>
    /// Current count of maximum selectable objects.
    /// </summary>
    private int m_CurrentMaximumSelectedObjects = 3;

    /// <summary>
    /// Private reference to all UI elements.
    /// </summary>
    private Dictionary<string, GameObject> m_UI_Elements = new Dictionary<string, GameObject>();

    /// <summary>
    /// I am not sure what this is. Will be deleted soon.
    /// </summary>
    private Material m_UI_Line_Material;

    /// <summary>
    /// Initializes all variables.
    /// </summary>
    /// <returns></returns>
    IEnumerator Start()
    {
        m_CurrentMaximumSelectedObjects = m_MaximumSelectableObjects;

        // Find roboy
        m_Roboy = RoboyManager.Instance.Roboy;

        // If you dont find roboy, quit
        if (m_Roboy == null)
            yield break;

        // Initiliaze roboy list
        foreach (Transform t in m_Roboy)
        {
            SelectableObject obj;
            if((obj = t.GetComponent<SelectableObject>()) != null)
                m_RoboyParts.Add(obj);
        }

        // Wait until you find all UI elements /// CHANGE THIS IN FUTURE TO CHECK FROM A SINGLETON? INSTANCE OR DATA STRUCTURE
        while (GameObject.FindGameObjectsWithTag("RoboyUI").Length < RoboyUIElementsCount)
            yield return null;

        GameObject[] tempUI_Elements = GameObject.FindGameObjectsWithTag("RoboyUI");

        // This part does not always work! When The Left Controller is not active then the attached RoboyUI objects cannot be found as they are inactive as well
        // => UI dictionary is empty => selectableObject cannot find UI! ------------- MAYBE FIXED!!!!----------------------
        // Add an Outline component to all UI elements if they dont have one
        foreach (GameObject g in tempUI_Elements)
        {
            m_UI_Elements.Add(g.name, g);

            Outline outline;

            if (g.GetComponent<Outline>() == null)
                g.AddComponent<Outline>();
            outline = g.GetComponent<Outline>();

            outline.enabled = false;
            outline.effectColor = Color.red;
            outline.effectDistance = new Vector2(0.2f, -0.2f);

            g.GetComponentInChildren<Text>().text = g.name;
        }

        // DONT KNOW WHAT THIS IS DELETE MAYBE
        m_UI_Line_Material = Resources.Load("mat_UI_Line", typeof(Material)) as Material;
    }

    /// <summary>
    /// Adds the roboy part to selected objects.
    /// </summary>
    /// <param name="obj">SelectableObject component of the roboy part.</param>
    public void AddSelectedObject(SelectableObject obj)
    {
        // If maximum number of possible selectable objects is exceeded, then unselect the oldest roboy part
        if (m_SelectedParts.Count >= m_CurrentMaximumSelectedObjects)
        {
            SelectableObject tmp = m_SelectedParts[0];
            tmp.SetStateDefault(true);
            m_SelectedParts.RemoveAt(0);         
        }

        // If the object is not yet selected, then select it
        if (!m_SelectedParts.Contains(obj))
            m_SelectedParts.Add(obj);
    }

    /// <summary>
    /// Removes the roboy part from the selected objects.
    /// </summary>
    /// <param name="obj">SelectableObject component of the roboy part.</param>
    public void RemoveSelectedObject(SelectableObject obj)
    {
        if (m_SelectedParts.Contains(obj))
            m_SelectedParts.Remove(obj);
    }

    /// <summary>
    /// Resets all roboy parts to default state and empties the selected objects list.
    /// </summary>
    public void ResetSelectedObjects()
    {
        foreach (SelectableObject obj in m_SelectedParts)
        {
            obj.SetStateDefault(true);
        }
        m_SelectedParts.Clear();
    }
}
