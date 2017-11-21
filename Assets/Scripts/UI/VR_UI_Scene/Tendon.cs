using UnityEngine;

/// <summary>
/// Object that represents one tendon.
/// Contains all attributes and needed functions.
/// </summary>
public class Tendon : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Tendon colors to interpolate between (only/exactly 3 used)
    /// </summary>
    private static Color[] m_TendonColors = { Color.green, Color.yellow, Color.red };

    /// <summary>
    /// Material for each linerenderer (no static one possible since color changes shall only affect one tendon).
    /// Call GetGraphMaterial for reference.
    /// </summary>
    private Material m_GraphMaterial;


    /// <summary>
    /// Maximum force value which can be applied to a tendon (for coloring purposes)
    /// </summary>
    private float m_MaxForce;

    /// <summary>
    /// Array containing all wirepoints of respective tendon
    /// </summary>
    private Transform[] m_Wirepoints;

    /// <summary>
    /// Array of all gameobjects which move the respective wirepoint
    /// </summary>
    private GameObject[] m_WirePointParents;

    /// <summary>
    /// force which is applied on the tendon
    /// </summary>
    private float m_Force;

    /// <summary>
    /// colour which is a result of the applied force
    /// </summary>
    private Color m_TendonColour;

    /// <summary>
    /// ID of this tendon
    /// </summary>
    private int m_TendonID;

    /// <summary>
    /// Line Renderer used for this tendon
    /// </summary>
    private LineRenderer m_LineRenderer;

    /// <summary>
    /// Needed to color tendons individually
    /// </summary>
    MaterialPropertyBlock m_Block;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// Updates tendons as soon as position(or other attributes) of tendons changed
    /// </summary>
    void Update()
    {
        if (m_Wirepoints == null) return;
        for (int i = 0; i < m_Wirepoints.Length; i++)
        {
            if (m_Wirepoints[i] && m_Wirepoints[i].hasChanged)
            {
                UpdateTendons();
                for (int j = i; j < m_Wirepoints.Length; j++)
                {
                    m_Wirepoints[j].hasChanged = false;
                }
                break;
            }
        }
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Returns unique ID of this tendon
    /// </summary>
    /// <returns></returns>
    public int GetTendonID()
    {
        return m_TendonID;
    }

    /// <summary>
    /// sets Maximum force (reference value) for one tendon
    /// </summary>
    /// <param name="maxForce"></param>
    public void SetMaxForce(float maxForce)
    {
        if (maxForce > 0)
            m_MaxForce = maxForce;
    }

    /// <summary>
    /// Initialises tendon with all points linked to the respective ObjectNames and the provided id.
    /// </summary>
    /// <param name="tendonID">ID of the tendon</param>
    /// <param name="pointPositions">positions of each tendon point in worldspace</param>
    /// <param name="objectNames">list of objects the points shall be linked to</param>
    /// <param name="maxforce">maximal Force which is applied to the tendon</param>
    public void Initialize(int tendonID, Vector3[] pointPositions, string[] objectNames, float maxforce)
    {
        m_TendonID = tendonID;
        m_MaxForce = maxforce;
        m_Force = 0;
        m_WirePointParents = new GameObject[pointPositions.Length];
        m_Wirepoints = new Transform[pointPositions.Length];
        m_Block = new MaterialPropertyBlock();

        //create points in worldspace and connect to parents
        for (int i = 0; i < pointPositions.Length; i++)
        {
            GameObject p = new GameObject();
            p.name = "Tendon " + tendonID + ": Wirepoint";
            p.transform.position = pointPositions[i];
            Transform parent = FindParent(objectNames[i]);
            if (!parent)
            {
                Debug.Log("[Tendon] Parent with name " + objectNames[i] + " could not be located. Tendons won't move with obj.");
                p.transform.SetParent(VRUILogic.Instance.GetDefaultWirepointContainer().transform);
            }
            else
            {
                p.transform.SetParent(parent);
                m_WirePointParents[i] = p.transform.parent.gameObject;
                m_Wirepoints[i] = p.transform;
            }
        }
        m_LineRenderer = gameObject.AddComponent<LineRenderer>();
        m_LineRenderer.useWorldSpace = true;
        m_LineRenderer.positionCount = pointPositions.Length;
        m_LineRenderer.material = GetGraphMaterial();
        m_LineRenderer.endWidth = 0.006f;
        m_LineRenderer.startWidth = 0.006f;

        if (m_MaxForce == 0)
        {
            m_MaxForce = 5; //TODO: guess for now!!
        }
        UpdateTendonColor();
        UpdateTendons();
    }

    /// <summary>
    /// updates the currently applied force
    /// </summary>
    /// <param name="force">Currently applied force</param>
    public void UpdateTendonForce(float force)
    {
        if (force > 0)
            m_Force = force;
        UpdateTendonColor();
    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// returns gameObject with respective string name. 
    /// </summary>
    /// <param name="parentName">name of the object to return</param>
    /// <returns></returns>
    private Transform FindParent(string parentName)
    {
        //first have a look in existing list of parents (very likely to contain obj)
        foreach (GameObject obj in m_WirePointParents)
        {
            if (obj && obj.name.Equals(parentName))
                return obj.transform;
        }
        GameObject parent = GameObject.Find(parentName);
        if (parent) return parent.transform;
        return null;
    }

    /// <summary>
    /// Draw lines of tendons with Linerenderer
    /// </summary>
    private void UpdateTendons()
    {
        if (m_Wirepoints == null)
            return;
        Vector3[] positions = new Vector3[m_Wirepoints.Length];
        for (int i = 0; i < m_Wirepoints.Length; i++)
        {
            if (m_Wirepoints[i])
                positions[i] = m_Wirepoints[i].position;
        }
        m_LineRenderer.SetPositions(positions);
    }

    /// <summary>
    /// Update tendon color based on current force
    /// </summary>
    private void UpdateTendonColor()
    {
        Color c;
        float normalizedForce = m_Force / m_MaxForce;
        if (normalizedForce > 1) normalizedForce = 1;
        if (normalizedForce < 0) normalizedForce = 0;
        normalizedForce *= normalizedForce; //stretch curve in a way where higher values make more difference
        //calculate color
        if (normalizedForce <= 0.5f)
        {
            normalizedForce *= 2;
            c = m_TendonColors[0] * (1 - normalizedForce) + m_TendonColors[1] * (normalizedForce);
        }
        else
        {
            normalizedForce -= 0.5f;
            normalizedForce *= 2;
            c = m_TendonColors[1] * (1 - normalizedForce) + m_TendonColors[2] * (normalizedForce);
        }
        //apply color
        /* TODO: not sure whether needed or not, works fine without these lines of code as well
        m_LineRenderer.GetPropertyBlock(block);
        block.SetColor("Color", c);
        m_LineRenderer.SetPropertyBlock(block);*/
        GetGraphMaterial().color = c;
    }

    /// <summary>
    /// Returns material, instantiates new TendonShader if none existing yet.
    /// </summary>
    /// <returns>material that graph uses to render</returns>
    private Material GetGraphMaterial()
    {
        if (!m_GraphMaterial)
        {
            //edit->projectsettings->graphics->always included shader must contain elem
            m_GraphMaterial = new Material(Shader.Find("Custom/TendonShader"));
        }
        return m_GraphMaterial;
    }
    #endregion
}

