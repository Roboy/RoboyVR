using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
[RequireComponent(typeof(GreenScreenManager))]
[RequireComponent(typeof(Camera))]
public class ZEDGarbageMatte : MonoBehaviour
{
    // Reference to the ZED
    private sl.ZEDCamera zed;

    //Position in queue to make transparent object, used to render the mesh transparent
    private const int QUEUE_TRANSPARENT_VALUE = 3000;

    //List of points to make the mesh
    private List<Vector3> points = new List<Vector3>();

    // The current camera looking the scene, used to transform ScreenPosition to worldPosition
    private Camera cam;

    //List of the different gameObjects used by the mesh
    private List<GameObject> go = null;

    //List of the meshes
    private List<MeshFilter> meshFilters;

    //Triangles of the current mesh
    private List<int> triangles = new List<int>();
    private List<GameObject> spheresBorder = new List<GameObject>();

    private Material shader_greenScreen;
    private Material outlineMaterial;
    private bool isClosed = false;

    private int currentPlaneIndex;
    private int sphereLayer = 21;

    private CommandBuffer commandBuffer;

    /*** OPTIONS TO CHANGE ****/
    //At launch, if no file are found, the GarbageMatte controls are activated if true. Button Fire1 || Fire2
    private bool isAbleToEdit = true;
    // Apply the garbageMatte is per default on the button "return"
    private string applyButton = "return";

    [SerializeField]
    [HideInInspector]
    public string garbageMattePath = "garbageMatte.cfg";

    [SerializeField]
    [HideInInspector]
    public bool editMode = true;

    [SerializeField]
    [HideInInspector]
    public bool loadAtStart = false;
    void Awake()
    {
        currentPlaneIndex = 0;
        zed = sl.ZEDCamera.GetInstance();
        cam = GetComponent<Camera>();
        points.Clear();

        outlineMaterial = Resources.Load("Materials/Mat_ZED_Outlined") as Material;

        go = new List<GameObject>();
        meshFilters = new List<MeshFilter>();
        if (shader_greenScreen == null)
        {
            shader_greenScreen = GetComponent<TextureOverlay>().canvas.GetComponent<Renderer>().material;
        }

        commandBuffer = new CommandBuffer();
        commandBuffer.name = "GarbageMatte";
        commandBuffer.SetRenderTarget(BuiltinRenderTextureType.CurrentActive, BuiltinRenderTextureType.Depth);
        cam.AddCommandBuffer(CameraEvent.BeforeDepthTexture, commandBuffer);

        if (loadAtStart && Load())
        {
            Debug.Log("Config garbage matte found, and loaded ( " + garbageMattePath + " )");
            ApplyGarbageMatte();
            editMode = false;
        }
    }

    private void PadReady()
    {
        ResetPoints();
        if (Load())
        {
            Debug.Log("Config garbage matte found, and loaded ( " + garbageMattePath + " )");
            ApplyGarbageMatte();
            editMode = false;
        }
        else
        {
            if(isAbleToEdit)
            {
                editMode = true;
            }

        }

    }

    private void OnEnable()
    {
        ZEDPadManager.ZEDOnPadIndexSet += PadReady;
    }

    private void OnDisable()
    {
        ZEDPadManager.ZEDOnPadIndexSet -= PadReady;

    }

    private int indexSelected = -1;
    private GameObject currentGOSelected = null;
    private MeshFilter meshFilterSelected;
    private int planeSelectedIndex = -1;

    // Update is called once per frame
    void Update()
    {
        if (editMode)
        {
            if (indexSelected != -1)
            {
                if (zed.CameraIsReady)
                {
                    Vector3 vec = cam.ScreenToWorldPoint(new Vector4(Input.mousePosition.x, Input.mousePosition.y, zed.GetDepthValue(Input.mousePosition)));
                    currentGOSelected.transform.position = vec;
                }
            }





            if (zed != null && zed.CameraIsReady)
            {
                if (Input.GetMouseButtonDown(0))
                {
                    if (go.Count - 1 < currentPlaneIndex)
                    {

                        go.Add(CreateGameObject());

                        go[currentPlaneIndex].GetComponent<MeshRenderer>().material.renderQueue = QUEUE_TRANSPARENT_VALUE + 5;
                        meshFilters[currentPlaneIndex] = go[currentPlaneIndex].GetComponent<MeshFilter>();
                        meshFilters[currentPlaneIndex].sharedMesh = CreateMesh();
                        meshFilters[currentPlaneIndex].sharedMesh.MarkDynamic();
                    }


                    if (indexSelected != -1)
                    {
                        indexSelected = -1;
                        currentGOSelected.GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.00f);
                        currentGOSelected = null;
                        meshFilterSelected.mesh.Clear();
                        if ((spheresBorder.Count - planeSelectedIndex * 4) < 4)
                        {
                            planeSelectedIndex = -1;
                            meshFilterSelected = null;
                            return;
                        }
                        List<int> triangles = new List<int>();
                        List<Vector3> points = new List<Vector3>();
                        for (int i = planeSelectedIndex * 4; i < (planeSelectedIndex + 1) * 4; i++)
                        {
                            points.Add(spheresBorder[i].transform.position);
                        }

                        CloseShape(triangles, points, planeSelectedIndex);
                        planeSelectedIndex = -1;
                        meshFilterSelected = null;
                        return;
                    }

                    else if (points.Count < 100 && !isClosed)
                    {
                        Vector3 vec = cam.ScreenToWorldPoint(new Vector4(Input.mousePosition.x, Input.mousePosition.y, zed.GetDepthValue(Input.mousePosition)));
                        RaycastHit hit;
                        if (Physics.Raycast(transform.position, (vec - transform.position), out hit, 10, (1 << sphereLayer)))
                        {
                            int hitIndex = spheresBorder.IndexOf(hit.transform.gameObject);
                            vec = spheresBorder[hitIndex].transform.position;
                        }
                        points.Add(vec);

                        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        sphere.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                        sphere.hideFlags = HideFlags.HideInHierarchy;
                        sphere.GetComponent<MeshRenderer>().material = outlineMaterial;
                        sphere.GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.02f);

                        sphere.transform.position = points[points.Count - 1];
                        sphere.layer = sphereLayer;
                        spheresBorder.Add(sphere);
                        if (spheresBorder.Count >= 2)
                        {
                            spheresBorder[spheresBorder.Count - 2].GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.00f);
                        }

                        if (spheresBorder.Count % 4 == 0)
                        {
                            List<Vector3> points = new List<Vector3>();
                            for (int i = currentPlaneIndex * 4; i < (currentPlaneIndex + 1) * 4; i++)
                            {
                                points.Add(spheresBorder[i].transform.position);
                            }
                            CloseShape(triangles, points, currentPlaneIndex);
                            EndPlane();
                        }
                    }
                }
                if (Input.GetMouseButtonDown(1))
                {
                    if (indexSelected != -1) return;
                    Vector3 vec = cam.ScreenToWorldPoint(new Vector4(Input.mousePosition.x, Input.mousePosition.y, zed.GetDepthValue(Input.mousePosition)));
                    RaycastHit hit;
                    if (Physics.Raycast(transform.position, (vec - transform.position), out hit, 10, (1 << sphereLayer)))
                    {
                        int hitIndex = spheresBorder.IndexOf(hit.transform.gameObject);
                        vec = spheresBorder[hitIndex].transform.position;
                        indexSelected = hitIndex;
                        currentGOSelected = spheresBorder[hitIndex];
                        planeSelectedIndex = hitIndex / 4;
                        meshFilterSelected = meshFilters[planeSelectedIndex];
                        spheresBorder[spheresBorder.Count - 1].GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.00f);
                        currentGOSelected.GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.02f);
                    }
                    else
                    {
                        planeSelectedIndex = -1;
                        indexSelected = -1;
                    }

                }
            }


            if(Input.GetKeyDown(applyButton))
            {
                ApplyGarbageMatte();
            }
        }
    }

    /// <summary>
    /// End the current plane and increase the index of plane
    /// </summary>
    public void EndPlane()
    {
        currentPlaneIndex++;
        ResetDataCurrentPlane();
    }

    public void EnterEditMode()
    {
        if (isClosed)
        {
            foreach (GameObject s in spheresBorder)
            {
                s.SetActive(true);
            }
            if (shader_greenScreen != null)
            {
                Shader.SetGlobalInt("_ZEDStencilComp", 0);
            }
            for (int i = 0; i < go.Count; i++)
            {
                if (go[i] == null) continue;
                go[i].GetComponent<MeshRenderer>().sharedMaterial.SetFloat("alpha", 0.5f);
                go[i].GetComponent<MeshRenderer>().sharedMaterial.renderQueue = QUEUE_TRANSPARENT_VALUE + 5;
            }
            isClosed = false;
        }
    }

    public void RemoveLastPoint()
    {
        if (isClosed)
        {
            foreach (GameObject s in spheresBorder)
            {
                s.SetActive(true);
            }
            if (shader_greenScreen != null)
            {
                Shader.SetGlobalInt("_ZEDStencilComp", 0);
            }
            for (int i = 0; i < go.Count; i++)
            {
                if (go[i] == null) continue;
                go[i].GetComponent<MeshRenderer>().sharedMaterial.SetFloat("alpha", 0.5f);
                go[i].GetComponent<MeshRenderer>().sharedMaterial.renderQueue = QUEUE_TRANSPARENT_VALUE + 5;
            }
            isClosed = false;
        }
        if (spheresBorder.Count % 4 == 0 && currentPlaneIndex > 0)
        {
            Destroy(go[currentPlaneIndex - 1]);
            go.RemoveAll(item => item == null);
            meshFilters.RemoveAll(item => item == null);
            meshFilters[currentPlaneIndex - 1].sharedMesh.Clear();

            currentPlaneIndex--;

        }

        if (spheresBorder != null && spheresBorder.Count > 0)
        {

            DestroyImmediate(spheresBorder[spheresBorder.Count - 1]);
            spheresBorder.RemoveAt(spheresBorder.Count - 1);
            if (spheresBorder.Count % 4 == 0 && spheresBorder.Count > 0)
            {
                spheresBorder[spheresBorder.Count - 1].GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.02f);
            }

        }

    }

    private void ResetDataCurrentPlane()
    {
        points.Clear();
        triangles.Clear();
    }

    public void ResetPoints()
    {
        if (go == null) return;
        isClosed = false;
        if (shader_greenScreen != null)
        {
            Shader.SetGlobalInt("_ZEDStencilComp", 0);
        }
        currentPlaneIndex = 0;
        for (int i = 0; i < go.Count; i++)
        {
            DestroyImmediate(go[i]);
        }
        go.Clear();
        meshFilters.Clear();
        ResetDataCurrentPlane();
        if (spheresBorder != null)
        {
            foreach (GameObject s in spheresBorder)
            {
                DestroyImmediate(s);
            }
        }
        spheresBorder.Clear();
        commandBuffer.Clear();

    }

    /// <summary>
    /// Get orientation
    /// </summary>
    /// <param name="p1"></param>
    /// <param name="p2"></param>
    /// <param name="p"></param>
    /// <param name="X"></param>
    /// <param name="Y"></param>
    /// <returns></returns>
    private static int Orientation(Vector3 p1, Vector3 p2, Vector3 p, Vector3 X, Vector3 Y)
    {
        return (Vector3.Dot(p2, X) - Vector3.Dot(p1, X)) * (Vector3.Dot(p, Y) - Vector3.Dot(p1, Y)) - (Vector3.Dot(p, X) - Vector3.Dot(p1, X)) * (Vector3.Dot(p2, Y) - Vector3.Dot(p1, Y)) > 0 ? 1 : 0;
    }

    /// <summary>
    /// Ordering the points to draw a mesh
    /// </summary>
    /// <returns></returns>
    private List<int> OrderPoints(List<Vector3> points)
    {
        Vector3 normal = Vector3.Cross((points[1] - points[0]), (points[2] - points[0]));
        normal.Normalize();
        Vector3 X = new Vector3(-normal.y, normal.x, 0);
        X.Normalize();
        Vector3 Y = Vector3.Cross(X, normal);
        Y.Normalize();

        List<int> ordoredIndex = new List<int>();

        List<Vector3> convexHull = new List<Vector3>();
        float minX = Vector3.Dot(points[0], X);
        Vector3 p = points[0];
        for (int i = 0; i < points.Count; i++)
        {
            if (Vector3.Dot(points[i], X) < minX)
            {
                minX = Vector3.Dot(points[i], X);
                p = points[i];
            }
        }
        Vector3 currentTestPoint;
        for (int i = 0; i < 4; i++)
        {
            convexHull.Add(p);
            ordoredIndex.Add(points.IndexOf(p));
            currentTestPoint = points[0];
            for (int j = 0; j < points.Count; j++)
            {
                if ((currentTestPoint == p) || (Orientation(p, currentTestPoint, points[j], X, Y) == 1))
                {
                    currentTestPoint = points[j];
                }
            }
            p = currentTestPoint;
        }
        return ordoredIndex;
    }

    /// <summary>
    /// Draw the last quad
    /// </summary>
    public void CloseShape(List<int> triangles, List<Vector3> points, int currentPlaneIndex)
    {
        triangles.Clear();
        List<int> indexOrder = OrderPoints(points);

        triangles.Add(indexOrder[0]);
        triangles.Add(indexOrder[1]);
        triangles.Add(indexOrder[2]);
        triangles.Add(indexOrder[0]);
        triangles.Add(indexOrder[2]);
        triangles.Add(indexOrder[3]);

        if (go[currentPlaneIndex] == null)
        {
            go[currentPlaneIndex] = CreateGameObject();
            meshFilters[currentPlaneIndex] = go[currentPlaneIndex].GetComponent<MeshFilter>();
            meshFilters[currentPlaneIndex].sharedMesh = CreateMesh();
            meshFilters[currentPlaneIndex].sharedMesh.MarkDynamic();
        }
        go[currentPlaneIndex].GetComponent<MeshFilter>().sharedMesh.Clear();
        go[currentPlaneIndex].GetComponent<MeshFilter>().sharedMesh.vertices = points.ToArray();
        go[currentPlaneIndex].GetComponent<MeshFilter>().sharedMesh.triangles = triangles.ToArray();

        spheresBorder[spheresBorder.Count - 1].GetComponent<MeshRenderer>().material.SetFloat("_Outline", 0.00f);

    }

    /// <summary>
    /// Apply the garbage matte by rendering into the stencil buffer
    /// </summary>
    public void ApplyGarbageMatte()
    {

        if (currentPlaneIndex <= 0)
        {
            editMode = false;
            ResetPoints();
            return;
        }
        if (shader_greenScreen != null )
        {
            isClosed = true;
            foreach (GameObject s in spheresBorder)
            {
                s.SetActive(false);
            }
            Shader.SetGlobalInt("_ZEDStencilComp", 3);
            for (int i = 0; i < go.Count; i++)
            {
                if (go[i] == null) continue;
                go[i].GetComponent<MeshRenderer>().sharedMaterial.SetFloat("alpha", 0.0f);
                go[i].GetComponent<MeshRenderer>().sharedMaterial.renderQueue = QUEUE_TRANSPARENT_VALUE - 5;
            }
        }

        commandBuffer.Clear();
        for (int i = 0; i < go.Count; ++i)
        {
            if(go[i] != null)
            commandBuffer.DrawMesh(go[i].GetComponent<MeshFilter>().mesh, go[i].transform.localToWorldMatrix, go[i].GetComponent<Renderer>().material);
        }
        editMode = false;
    }

    private void OnApplicationQuit()
    {
        ResetPoints();
    }

    private GameObject CreateGameObject()
    {
        GameObject plane = new GameObject("PlaneTest");
        plane.hideFlags = HideFlags.HideInHierarchy;
        meshFilters.Add((MeshFilter)plane.AddComponent(typeof(MeshFilter)));

        MeshRenderer renderer = plane.AddComponent(typeof(MeshRenderer)) as MeshRenderer;
        renderer.sharedMaterial = new Material(Resources.Load("Materials/Mat_ZED_Mask_Quad") as Material);
        renderer.sharedMaterial.SetFloat("alpha", 0.5f);
        return plane;
    }

    private Mesh CreateMesh()
    {
        Mesh m = new Mesh();
        m.name = "ScriptedMesh";

        triangles.Add(0);
        triangles.Add(1);
        triangles.Add(2);

        return m;
    }

    /// <summary>
    /// Save the points into a file
    /// </summary>
    public void Save()
    {
        List<string> meshes = new List<string>();
        meshes.Add((meshFilters.Count - 1).ToString());

        for (int i = 0; i < meshFilters.Count - 1; i++)
        {
            Vector3[] vertices = meshFilters[i].mesh.vertices;
            int[] tri = meshFilters[i].mesh.triangles;
            meshes.Add("v#" + vertices.Length);
            for (int j = 0; j < vertices.Length; j++)
            {
                meshes.Add(vertices[j].x + " " + vertices[j].y + " " + vertices[j].z);
            }
        }
        System.IO.File.WriteAllLines(garbageMattePath, meshes.ToArray());
    }

    /// <summary>
    /// Load the current shape
    /// </summary>
    public bool Load()
    {
        if (!System.IO.File.Exists(garbageMattePath)) return false;
        string[] meshes = System.IO.File.ReadAllLines(garbageMattePath);
        if (meshes == null) return false;
        int nbMesh = int.Parse(meshes[0]);
        if (nbMesh < 0) return false;
        currentPlaneIndex = 0;
        ResetPoints();
        int lineCount = 1;
        string[] splittedLine;
        for (int i = 0; i < nbMesh; i++)
        {
            points.Clear();
            triangles.Clear();
            go.Add(CreateGameObject());
            go[currentPlaneIndex].GetComponent<MeshRenderer>().material.renderQueue = QUEUE_TRANSPARENT_VALUE + 5;
            meshFilters[currentPlaneIndex] = go[currentPlaneIndex].GetComponent<MeshFilter>();
            meshFilters[currentPlaneIndex].sharedMesh = CreateMesh();
            meshFilters[currentPlaneIndex].sharedMesh.MarkDynamic();
            splittedLine = meshes[lineCount].Split('#');
            lineCount++;
            int nbVertices = int.Parse(splittedLine[1]);
            for (int j = 0; j < nbVertices; j++)
            {
                splittedLine = meshes[lineCount].Split(' ');
                lineCount++;
                float x = float.Parse(splittedLine[0]), y = float.Parse(splittedLine[1]), z = float.Parse(splittedLine[2]);
                points.Add(new Vector3(x, y, z));
                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                sphere.hideFlags = HideFlags.HideInHierarchy;

                sphere.transform.position = points[points.Count - 1];
                sphere.layer = sphereLayer;
                spheresBorder.Add(sphere);
            }
            if (go.Count == 0) return false;

            CloseShape(triangles, points, currentPlaneIndex);
            EndPlane();
        }
        return true;
    }
}
