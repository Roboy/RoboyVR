using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class ModelManager : Singleton<ModelManager>

{

    #region PUBLIC_MEMBER_VARIABLES
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    [SerializeField]
    private List<GameObject> m_Models = null;

    [SerializeField]
    private GameObject m_Model = null;

    private int m_Index = 0;
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    // Use this for initialization
    void Start()
    {
        m_Models = new List<GameObject>(GameObject.FindGameObjectsWithTag("Roboy"));
        m_Model = m_Models[m_Index];
    }

    void Update()
    {
        //Erasing a model
        if (Input.GetKey(KeyCode.E))
        {
            if (Input.GetMouseButtonDown(0))
            {
                GameObject g = CheckForImpact();
                if (g != null)
                {
                    DeleteModel(g);
                }
            }
        }


        //Inserting a model
        if (Input.GetKey(KeyCode.I))
        {
            if (Input.GetMouseButtonDown(0))
            {
                Vector3 hit = CheckForSpace();
                if (hit != new Vector3(42.0f, 42.0f, 42.0f))
                {
                    hit.y += 1.0f;
                    AddModel(m_Models[m_Index].name, hit);
                }
            }
        }

        if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            if (m_Index < (m_Models.Count - 1))
            {
                m_Model = m_Models[m_Index + 1];
                m_Index += 1;
            }
            else {
                m_Model = m_Models[0];
                m_Index = 0;
            }
        }
        if (Input.GetKeyDown(KeyCode.DownArrow))
        {
            if (m_Index > 0)
            {
                
                m_Index -= 1;
                m_Model = m_Models[m_Index];
            }
            else
            {
                
                m_Index = m_Models.Count -1;
                m_Model = m_Models[m_Index];
            }

        }


    }
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS
    public string GetNameOfModel()
    {
        return m_Model.name;
    }

    public void AddModel(string s, Vector3 pos)
    {
        string path = "Assets/Prefabs/SimulationModels/" + s + ".prefab";
        GameObject obj = (GameObject)AssetDatabase.LoadAssetAtPath(path, typeof(GameObject));
        GameObject final = Instantiate(obj, pos, obj.transform.rotation);
        final.transform.LookAt(Vector3.zero);
        final.name = obj.name;
        m_Models.Add(final);
    }

    public void DeleteModel(GameObject obj)
    {
        if (m_Models.Contains(obj))
        {
            
            if (m_Index == (m_Models.Count - 1))
            {
                m_Index -= 1;
                m_Model = m_Models[m_Index];
            }

            m_Models.Remove(obj);

            if (m_Index < (m_Models.Count - 1))
            {
                m_Model = m_Models[m_Index];
            }
            Destroy(obj);
        }else{
            Debug.LogWarning("GameObject not present in Scene!");
        }
    }
    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS   
    private GameObject CheckForImpact()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        // Casts the ray and get the first game object hit

        if (Physics.Raycast(ray, out hit))
        {
            Transform t = hit.transform;
            //Try finding the root of the child
            while (t.parent != null) {
                t = t.parent;
            }
            return t.gameObject;
        }

        //if it hits nothing
        return null;
        
    }

    private Vector3 CheckForSpace()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        // Casts the ray and get the first game object hit

        if (Physics.Raycast(ray, out hit))
        {
            //If we hit the ground, return postion of hitpoint
            if (hit.transform.gameObject.tag == "Floor")
            {
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.transform.position = new Vector3(hit.point.x, hit.point.y+ 1.0f, hit.point.z);
                cube.transform.localScale = new Vector3(0.6f, 0.9f, 0.6f);
                cube.GetComponent<MeshRenderer>().enabled = false;
                Renderer r = cube.GetComponent<Renderer>();
                foreach (GameObject m in m_Models)
                {
                    if (r.bounds.Intersects(m.GetComponentInChildren<Renderer>().bounds))
                    {
                        //Do not insert here
                        //Send error vector
                        Destroy(cube);
                        return new Vector3(42.0f, 42.0f, 42.0f);
                    }
                }
                Destroy(cube);
                return hit.point;
            }
            
        }
        //If there is no hit
        return new Vector3(42.0f, 42.0f, 42.0f);

    }
    #endregion //PRIVATE_METHODS
}