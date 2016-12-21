using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.UI;

public class SelectorManager : Singleton<SelectorManager>
{

    public RectTransform Canvas;

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
            g.GetComponent<Image>().enabled = false;
            g.GetComponentInChildren<Text>().enabled = false;
            
        }

        m_UI_Line_Material = Resources.Load("mat_UI_Line", typeof(Material)) as Material;
    }

    void Update()
    {
        updateRays();
    }

    public void AddSelectedObject(SelectableObject obj)
    {
        if(!m_SelectedParts.Contains(obj))
            m_SelectedParts.Add(obj);
        
        m_UI_Elements[obj.name].GetComponent<Image>().enabled = true;
        m_UI_Elements[obj.name].GetComponentInChildren<Text>().enabled = true;

        Vector3 ui_pos = m_UI_Elements[obj.name].transform.position;
        ui_pos.z = Camera.main.nearClipPlane;

        Vector3 lrEndPosition = Camera.main.ScreenToWorldPoint(ui_pos);

        LineRenderer lr = obj.gameObject.AddComponent<LineRenderer>();
        lr.numPositions = 2;
        lr.startWidth = 0.005f;
        lr.SetPositions(new Vector3[] { obj.transform.position, lrEndPosition });
        lr.material = m_UI_Line_Material;


    }

    public void RemoveSelectedObject(SelectableObject obj)
    {
        if (m_SelectedParts.Contains(obj))
            m_SelectedParts.Remove(obj);

        m_UI_Elements[obj.name].GetComponent<Image>().enabled = false;
        m_UI_Elements[obj.name].GetComponentInChildren<Text>().enabled = false;

        Destroy(obj.GetComponent<LineRenderer>());
    }

    private void updateRays()
    {
        foreach (SelectableObject obj in m_SelectedParts)
        {
            LineRenderer lr = obj.GetComponent<LineRenderer>();

            lr.SetPosition(0, obj.transform.position);
        }
    }
}
