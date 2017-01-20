using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.UI;

public class SelectorManager : Singleton<SelectorManager>
{

    public RectTransform Canvas;

    public Dictionary<string, GameObject> UI_Elements { get { return m_UI_Elements; } }

    private Transform m_Roboy;

    private List<SelectableObject> m_RoboyParts = new List<SelectableObject>();

    private List<SelectableObject> m_SelectedParts = new List<SelectableObject>();

    private Dictionary<string, GameObject> m_UI_Elements = new Dictionary<string, GameObject>();

    private Material m_UI_Line_Material;

    void Awake()
    {
        m_Roboy = GameObject.FindGameObjectWithTag("Roboy").transform;

        if (m_Roboy == null)
            return;

        foreach (Transform t in m_Roboy)
        {
            SelectableObject obj;
            if((obj = t.GetComponent<SelectableObject>()) != null)
                m_RoboyParts.Add(obj);
        }

        GameObject[] tempUI_Elements = GameObject.FindGameObjectsWithTag("RoboyUI");

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

        m_UI_Line_Material = Resources.Load("mat_UI_Line", typeof(Material)) as Material;
    }

    public void AddSelectedObject(SelectableObject obj)
    {
        if (m_SelectedParts.Count >= 3)
        {
            SelectableObject tmp = m_SelectedParts[0];
            tmp.SetStateDefault(true);
            m_SelectedParts.RemoveAt(0);         
        }

        if (!m_SelectedParts.Contains(obj))
            m_SelectedParts.Add(obj);

        // m_UI_Elements[obj.name].GetComponent<Outline>().enabled = true;
    }

    public void RemoveSelectedObject(SelectableObject obj)
    {
        if (m_SelectedParts.Contains(obj))
            m_SelectedParts.Remove(obj);

        //m_UI_Elements[obj.name].GetComponent<Outline>().enabled = false;
    }
}
