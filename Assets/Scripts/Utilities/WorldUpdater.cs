using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using System.Globalization;

[ExecuteInEditMode]
public class WorldUpdater : MonoBehaviour
{
    struct LinkTransformation
    {
        public string name;
        public Vector3 position;
        public Vector3 rotation;
        public string VIS_meshName;
        public Vector3 VIS_scale;
        public string COL_meshName;
    }

    struct ModelTransformation
    {
        public string worldname;
        public string name;
        public Vector3 position;
        public Vector3 rotation;
        public LinkTransformation link;

    }

    /// <summary>
    /// Github repository of the roboy models.
    /// </summary>
    public string Github_Repository = @"https://github.com/Roboy/roboy_worlds/";

    public string Branch = "master";

    /// <summary>
    /// Public property for the editor script
    /// </summary>
    [HideInInspector]
    public UpdaterUtility.State WorldsCurrentState = UpdaterUtility.State.None;

    private string m_PathToModelsFolder;


    /// <summary>
    /// Public property of the URL Dic for the editor script
    /// </summary>
    public Dictionary<string, string> URLDictionary { get { return m_URLDictionary; } }

    /// <summary>
    /// Dictionary to store the users choice whether he wants to update the model or not
    /// </summary>
    public Dictionary<string, bool> WorldChoiceDictionary = new Dictionary<string, bool>();

    /// <summary>
    /// Stores all model "Titles + URLs"
    /// </summary>
    private Dictionary<string, string> m_URLDictionary = new Dictionary<string, string>();

    private List<ModelTransformation> modellist = new List<ModelTransformation>();

    private Dictionary<string, GameObject> m_PrefabDictionary = new Dictionary<string, GameObject>();

    // Use this for initialization
    void Awake()
    {
        Initialize();
    }

    /// <summary>
    /// Initializes the paths of the python scripts.
    /// </summary>
    public void Initialize()
    {
        UpdaterUtility.ProjectFolder = Application.dataPath;
        UpdaterUtility.PathToDownloadScript = UpdaterUtility.ProjectFolder + @"/ExternalTools/ModelDownloader.py";
        UpdaterUtility.PathToScanScript = UpdaterUtility.ProjectFolder + @"/ExternalTools/ModelScanner.py";
        UpdaterUtility.PathToWorldReader = UpdaterUtility.ProjectFolder + @"/ExternalTools/world_reader.py";

        UpdaterUtility.showWarnings();
    }

    /// <summary>
    /// Scans the roboy_worlds repository and stores the worlds in a dictionary.
    /// </summary>
    public void Scan()
    {
        string[] scanArguments = { "python", UpdaterUtility.PathToScanScript, Github_Repository + @"tree/" + Branch };
        CommandlineUtility.RunCommandLine(scanArguments);
        // to do whether scan file exists and is right
        // check whether file exists
        string pathToScanFile = UpdaterUtility.ProjectFolder + @"/tempModelURLs.txt";
        if (!File.Exists(pathToScanFile))
        {
            Debug.LogWarning("Scan file not found! Check whether it exists or if python script is working!");
            return;
        }
        // get file content of format title;url
        string[] scanContent = File.ReadAllLines(pathToScanFile);

        File.Delete(pathToScanFile);

        Dictionary<string, string> tempURLDic = new Dictionary<string, string>();
        foreach (var line in scanContent)
        {
            // split at ":"
            string[] titleURL = line.Split(';');
            // check if there is exactly one ";" meaning only two elements
            if (titleURL.Length != 2)
            {
                Debug.Log("In line:\n" + line + "\nthe format does not match title;URL");
                continue;
            }
            // ignore link if it is not in the github repo
            if (!titleURL[1].Contains(Github_Repository))
            {
                Debug.Log("Link does not have the github repository!");
                continue;
            }
            if (titleURL[0] != "models")
            {
                tempURLDic.Add(titleURL[0], titleURL[1]);
            }
            else
            {
                m_PathToModelsFolder = titleURL[1];
            }
        }
        // clear all old links and add the new links
        m_URLDictionary.Clear();
        m_URLDictionary = tempURLDic;
        WorldChoiceDictionary.Clear();

        Debug.Log(m_PathToModelsFolder);

        //this will be used to select the models to download
        foreach (var urlDicEntry in m_URLDictionary)
        {
            WorldChoiceDictionary.Add(urlDicEntry.Key, false);
        }
        WorldsCurrentState = UpdaterUtility.State.Scanned;
    }


    public void LoadWorlds()
    {
        string pathToOriginWorlds = UpdaterUtility.ProjectFolder + @"/SimulationWorlds/";

        List<KeyValuePair<string, bool>> tempURLList = WorldChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {
            //replace "tree" with "raw" in URL
            var regex = new Regex(Regex.Escape("tree"));
            m_URLDictionary[urlEntry.Key] = regex.Replace(m_URLDictionary[urlEntry.Key], "raw", 1);
            //start modeldownloader.py for .world files
            string[] updateArgumentsWorld = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, m_URLDictionary[urlEntry.Key] + @"/", UpdaterUtility.ProjectFolder + @"/SimulationWorlds/" + urlEntry.Key, "" };
            CommandlineUtility.RunCommandLine(updateArgumentsWorld);
        }
    }

    public void Magic()
    {

        List<KeyValuePair<string, bool>> tempURLList = WorldChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {   //replace this with real data from .world files

            if (File.Exists(UpdaterUtility.ProjectFolder + @"/SimulationWorlds/" + urlEntry.Key + @"/" + urlEntry.Key + ".world"))
            {
                Debug.Log(".world file found!");
                // read .sdf file
                string[] argumentsSDFreader = { "python \"" + UpdaterUtility.PathToWorldReader + "\" \"" + UpdaterUtility.ProjectFolder + @"/SimulationWorlds/" + urlEntry.Key + @"/" + urlEntry.Key + ".world\"" };
                CommandlineUtility.RunCommandLine(argumentsSDFreader);
            }
            else
            {
                Debug.LogWarning(".world file not found!");
            }

            string pathToWorldFile = UpdaterUtility.ProjectFolder + @"/temp" + urlEntry.Key + @"World.txt";
            if (!File.Exists(pathToWorldFile))
            {
                //Debug.LogWarning(UpdaterUtility.ProjectFolder + @"/temp" + urlEntry.Key + @"World.txt");
                Debug.LogWarning("Scan file not found! Check whether it exists or if python script is working!");
                return;
            }
            // get file content of format title:url
            string[] WorldContent = File.ReadAllLines(pathToWorldFile);

            //File.Delete(pathToWorldFile);

            List<string[]> modelList = new List<string[]>();

            foreach (string line in WorldContent)
            {
                string[] SDFline = line.Split(';');
                modelList.Add(SDFline);

            }
            string worldName = null;
            for (int i = 0; i < modelList.Count; i++)
            //foreach (string[] line in linkList)
            {
                if (modelList[i][0] == "world_name")
                {
                    //Create GameObject where everything will be attached
                    worldName = modelList[i][1];
                    modelList.Remove(modelList[i]);
                    //continue;
                }
            }

            //for (int i = 0; i < modelList.Count; i++)
            //{
                foreach (string[] line in modelList)
                {
                    ModelTransformation testModel = new ModelTransformation();
                    testModel.worldname = worldName;
                    for (int j = 0; j < line.Length; j++)
                    {
                        testModel.link = new LinkTransformation();

                        string[] pose = null;
                        if (line[j] == "model_pose")
                        {
                            pose = line[j + 1].Split(' ');
                            testModel.position = GazeboUtility.GazeboPositionToUnity(new Vector3(float.Parse(pose[0], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                float.Parse(pose[1], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                float.Parse(pose[2], CultureInfo.InvariantCulture.NumberFormat)));
                            testModel.rotation = GazeboUtility.GazeboPositionToUnity(new Vector3(Mathf.Rad2Deg * float.Parse(pose[3], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 Mathf.Rad2Deg * float.Parse(pose[4], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 Mathf.Rad2Deg * float.Parse(pose[5], CultureInfo.InvariantCulture.NumberFormat)));
                        }                        

                        if (line[j] == "model_link")
                        {
                            testModel.link.name = line[j + 1];
                        }
                        if (line[j] == "VIS_mesh_uri") {
                            testModel.link.VIS_meshName = line[j + 1];
                            //string name1 = modelList[i][j + 1];
                            string name1 = Regex.Replace(line[j + 1], ".*?//", "");
                            string name2 = Regex.Replace(name1, "/.*", "");
                            testModel.name = name2;
                            Debug.Log(testModel.link.VIS_meshName);
                        }

                        if (line[j] == "COL_mesh_uri")
                            testModel.link.COL_meshName = line[j + 1];

                        string[] VIS_Scale = null;
                        if (line[j] == "VIS_mesh_scale")
                        {
                            VIS_Scale = line[j + 1].Split(' ');
                        }
                        if (VIS_Scale != null)
                        {
                            testModel.link.VIS_scale = GazeboUtility.GazeboPositionToUnity(new Vector3(100 * float.Parse(VIS_Scale[0], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                            100 * float.Parse(VIS_Scale[1], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                           100 * float.Parse(VIS_Scale[2], CultureInfo.InvariantCulture.NumberFormat)));
                        }
                        string[] linkpose = null;
                        if (line[j] == "link_pose")
                        {
                            pose = line[j + 1].Split(' ');
                        }
                        if (linkpose != null)
                        {
                            testModel.position = GazeboUtility.GazeboPositionToUnity(new Vector3(float.Parse(linkpose[0], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 float.Parse(linkpose[1], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 float.Parse(linkpose[2], CultureInfo.InvariantCulture.NumberFormat)));
                            testModel.rotation = GazeboUtility.GazeboPositionToUnity(new Vector3(Mathf.Rad2Deg * float.Parse(linkpose[3], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 Mathf.Rad2Deg * float.Parse(linkpose[4], CultureInfo.InvariantCulture.NumberFormat),
                                                                                                 Mathf.Rad2Deg * float.Parse(linkpose[5], CultureInfo.InvariantCulture.NumberFormat)));
                        }
                        
                    }
                    modellist.Add(testModel);


                //}
            }


            WorldsCurrentState = UpdaterUtility.State.Downloaded;
        }
        List<string> names = new List<string>();
        foreach (ModelTransformation model in modellist)
        {
            if (!names.Contains(model.name))
            {
                names.Add(model.name);
                //Debug.Log(model.name);
            }
        }
        foreach (string name in names)
        {

            //replace "tree" with "raw" in URL
            var regex = new Regex(Regex.Escape("tree"));
            m_PathToModelsFolder = regex.Replace(m_PathToModelsFolder, "raw", 1);
            //start modeldownloader.py for visual
            string[] updateArgumentsWorld = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, m_PathToModelsFolder + @"/" + name + @"/meshes/", UpdaterUtility.ProjectFolder + @"/SimulationWorlds/Models/" + name + "/meshes", "" };
            CommandlineUtility.RunCommandLine(updateArgumentsWorld);
        }
    }


    public void CreateWorld()
    {
        //stores every individual model in a list, to later create prefabs
        List<string> modelNames = new List<string>();
        foreach (ModelTransformation model in modellist)
        {
            if (!modelNames.Contains(model.name))
            {
                modelNames.Add(model.name);
            }
        }
        //Create Prefab for individual models
        foreach (string modelName in modelNames)
        {
            string absoluteModelPath = UpdaterUtility.ProjectFolder + @"/SimulationWorlds/Models/" + modelName + @"/meshes";
            //Create GameObject where everything will be attached
            GameObject modelParent = new GameObject(modelName);

            //List for all downloaded visuals
            List<string> MeshList = new List<string>();
            if (absoluteModelPath != "")
            {
                MeshList = UpdaterUtility.getFilePathsFBX(absoluteModelPath);
            }

            foreach (string name in MeshList)
            {
                GameObject meshPrefab = null;
                string relativeModelPath = @"Assets/SimulationWorlds/Models/" + modelName + @"/meshes/";
                //Debug.Log(relativeModelPath + name);

                // import Mesh (from visual if we changed folder structure of roboy_worlds/models
                meshPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(relativeModelPath + name, typeof(Object));

                if (meshPrefab == null)
                {
                    Debug.Log("Could not import model!");
                    continue;
                }

                GameObject meshCopy = Instantiate(meshPrefab);
                // set scale of childs to (1,1,1) in case the .stl file has childs with different scale
                foreach (Transform child in meshCopy.transform)
                {
                    child.localScale = Vector3.one;
                }
                meshCopy.transform.localScale = Vector3.one;

                //foreach (ModelTransformation model in modellist)
                //{
                //    string name1 = name.Replace(".fbx", "");
                //    if (model.link.VIS_meshName.Contains(name1))
                //    {
                //        if (model.link.position != null)
                //        {
                //            meshCopy.transform.position = model.link.position;
                //        }
                //        if (model.link.rotation != null)
                //        {
                //            meshCopy.transform.eulerAngles = model.link.rotation;
                //        }
                //        if (model.link.VIS_scale != null)
                //        {
                //            meshCopy.transform.localScale = model.link.VIS_scale;
                //        }
                //    }
                //}

                //meshCopy.tag = "WorldPart";
                // !!!! the following part could be shortened if we change folder structure of roboy_worlds/models
                //UpdaterUtility.attachCollider(meshCopy, relativeModelPath, name);

                // get the object which serves as mesh collider
                GameObject colliderPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(relativeModelPath + name, typeof(Object));

                if (colliderPrefab == null)
                {
                    Debug.Log("Did not find a collider object for mesh: " + name);
                    return;
                }
                // set scale of childs to (1,1,1) in case the .stl file has childs with different scale
                foreach (Transform child in colliderPrefab.transform)
                {
                    child.localScale = Vector3.one;
                }
                colliderPrefab.transform.localScale = Vector3.one;

                // go through each mesh filter and add all mesh references as mesh colliders to the gameObject
                List<MeshFilter> collRenderers = colliderPrefab.GetComponentsInChildren<MeshFilter>().ToList();


                foreach (MeshFilter collRenderer in collRenderers)
                {
                    MeshCollider meshCollider = meshCopy.AddComponent<MeshCollider>();
                    meshCollider.sharedMesh = collRenderer.sharedMesh;
                }

                var regex1 = new Regex(Regex.Escape("(Clone)"));
                meshCopy.name = regex1.Replace(meshCopy.name, "", 1);

                //var regex2 = new Regex(Regex.Escape("VIS_"));
                //meshCopy.name = regex2.Replace(meshCopy.name, "", 1);

                // Attach Model with mesh to parent GO
                meshCopy.transform.parent = modelParent.transform;
            }

            //Create Prefab of existing GO
            Object prefab = PrefabUtility.CreateEmptyPrefab("Assets/SimulationWorlds/Models/" + modelName + "/" + modelName + ".prefab");
            GameObject test1 = PrefabUtility.ReplacePrefab(modelParent, prefab, ReplacePrefabOptions.ConnectToPrefab);
            //add to dictionary to instanciate later
            m_PrefabDictionary.Add(modelName, test1);
            //Destroy GO after prefab is created
            DestroyImmediate(modelParent);
        }
        List<KeyValuePair<string, bool>> tempURLList = WorldChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {
            //this is the parent object of the entire world
            GameObject worldParent = new GameObject(urlEntry.Key);
            //foreach model in the .world files
            foreach (ModelTransformation model in modellist)
            {
                string absoluteWorldPath = UpdaterUtility.ProjectFolder + @"/SimulationWorlds/" + model.name;

                //if model is part of the world urlEntry.Key
                if (model.worldname == urlEntry.Key)
                {
                    GameObject model1 = Instantiate(m_PrefabDictionary[model.name]);
                    var regex1 = new Regex(Regex.Escape("(Clone)"));
                    model1.name = regex1.Replace(model1.name, "", 1);
                    //set transform with the values gathered in .world file
                    model1.transform.position = model.position;
                    model1.transform.eulerAngles = model.rotation;
                    //model1.transform.localScale = model.scale;
                    model1.transform.parent = worldParent.transform;
                }

            }
            // save whole world as prefab
            Object prefab = PrefabUtility.CreateEmptyPrefab("Assets/SimulationWorlds/" + urlEntry.Key + "/" + urlEntry.Key + ".prefab");
            GameObject test1 = PrefabUtility.ReplacePrefab(worldParent, prefab, ReplacePrefabOptions.ConnectToPrefab);
            //Destroy GO after prefab is created
            DestroyImmediate(worldParent);
        }
    }
}

