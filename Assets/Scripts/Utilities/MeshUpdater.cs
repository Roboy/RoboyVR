using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;

[ExecuteInEditMode]
public class MeshUpdater : MonoBehaviour
{

    /// <summary>
    /// Github repository of the roboy models.
    /// </summary>
    public string Github_Repository = @"https://github.com/Roboy/roboy_models/";

    public string Branch = "VRTeam";


    /// <summary>
    /// Public property for the editor script
    /// </summary>
    [HideInInspector]
    public UpdaterUtility.State ModelsCurrentState = UpdaterUtility.State.None;

    /// <summary>
    /// Public property of the URL Dic for the editor script
    /// </summary>
    public Dictionary<string, string> URLDictionary { get { return m_URLDictionary; } }

    /// <summary>
    /// Dictionary to store the users choice whether he wants to update the model or not
    /// </summary>
    public Dictionary<string, bool> ModelChoiceDictionary = new Dictionary<string, bool>();

    /// <summary>
    /// Stores all model "Titles + URLs"
    /// </summary>
    private Dictionary<string, string> m_URLDictionary = new Dictionary<string, string>();

    /// <summary>
    /// Temp ModelName 
    /// </summary>
    private List<string> m_ModelNames = new List<string>();


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
        UpdaterUtility.PathToSDFreader = UpdaterUtility.ProjectFolder + @"/ExternalTools/SDF_reader.py";

        UpdaterUtility.showWarnings();
    }

    /// <summary>
    /// Scans the repository for the roboy models and stores them in a dictionary.
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

            tempURLDic.Add(titleURL[0], titleURL[1]);
        }
        // clear all old links and add the new links
        m_URLDictionary.Clear();
        m_URLDictionary = tempURLDic;
        ModelChoiceDictionary.Clear();

        //this will be used to select the models to download
        foreach (var urlDicEntry in m_URLDictionary)
        {
            ModelChoiceDictionary.Add(urlDicEntry.Key, false);
        }
        ModelsCurrentState = UpdaterUtility.State.Scanned;
    }

    /// <summary>
    /// Downloads the models from the scan dictionary which were selected by the user.
    /// </summary>
    public void UpdateModels()
    {
        string pathToOriginModels = UpdaterUtility.ProjectFolder + @"/SimulationModels/";
        //var processInfo = new ProcessStartInfo("cmd.exe", "/C" + "start \"\" \"" + m_PathToBlender + "\" -P \"" + m_PathToDownloadScript + "\" \"" + pathToMeshes + "\" \"" + m_pathToProjectModels + "\" \"\"");
        //UnityEngine.Debug.Log("Run not implemented yet!");
        // get a list of all entries which the user wants to update
        List<KeyValuePair<string, bool>> tempURLList = ModelChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {   
            string[] scanArguments = { "python", UpdaterUtility.PathToScanScript, m_URLDictionary[urlEntry.Key] };
            CommandlineUtility.RunCommandLine(scanArguments);
            //Debug.Log(m_URLDictionary[urlEntry.Key]);

            string pathToScanFile = UpdaterUtility.ProjectFolder + @"/tempModelURLs.txt";
            if (!File.Exists(pathToScanFile))
            {
                Debug.LogWarning("Scan file not found! Check whether it exists or if python script is working!");
                return;
            }
            // get file content of format title:url
            string[] scanContent = File.ReadAllLines(pathToScanFile);

            File.Delete(pathToScanFile);

            foreach (var line in scanContent)
            {
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

                //replace "tree" with "raw" in URL
                var regex = new Regex(Regex.Escape("tree"));
                titleURL[1] = regex.Replace(titleURL[1], "raw", 1);

                //replace "tree" with "raw" in URL
                var regex1 = new Regex(Regex.Escape("tree"));
                string tempModelURL = regex.Replace(m_URLDictionary[urlEntry.Key], "raw", 1);

                //start modeldownloader.py for visual
                string[] updateArgumentsXML = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, tempModelURL + @"/", UpdaterUtility.ProjectFolder + @"/SimulationModels/" + urlEntry.Key + @"/OriginModels", "" };
                CommandlineUtility.RunCommandLine(updateArgumentsXML);

                //start modeldownloader.py for visual
                string[] updateArgumentsVis = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, titleURL[1] + @"/visual/", UpdaterUtility.ProjectFolder + @"/SimulationModels/" + urlEntry.Key + @"/OriginModels/visual", "" };
                CommandlineUtility.RunCommandLine(updateArgumentsVis);

                //start modeldownloader.py for collision
                string[] updateArgumentsCol = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, titleURL[1] + @"/collision/", UpdaterUtility.ProjectFolder + @"/SimulationModels/" + urlEntry.Key + @"/OriginModels/collision", "" };
                CommandlineUtility.RunCommandLine(updateArgumentsCol);

                if (!m_ModelNames.Contains(urlEntry.Key))
                {
                    m_ModelNames.Add(urlEntry.Key);
                }
            }
            if (File.Exists(pathToOriginModels + urlEntry.Key + @"/OriginModels/model.sdf"))
            {
                Debug.Log("model.sdf found!");
                // read .sdf file
                string[] argumentsSDFreader = { "python \"" + UpdaterUtility.PathToSDFreader + "\"", pathToOriginModels + urlEntry.Key + @"/OriginModels/model.sdf" };
                CommandlineUtility.RunCommandLine(argumentsSDFreader);
            }
            else
            {
                Debug.LogWarning("model.sdf not found!");
            }

            ModelsCurrentState = UpdaterUtility.State.Downloaded;
        }
    }

    /// <summary>
    /// Creates prefabs for every model which were downloaded. 
    /// </summary>
    public void CreatePrefab()
    {
        string pathToSDFFile = UpdaterUtility.ProjectFolder + @"/tempModelSDFs.txt";
        if (!File.Exists(pathToSDFFile))
        {
            Debug.LogWarning("Scan file not found! Check whether it exists or if python script is working!");
            return;
        }
        // get file content of format title:url
        string[] sdfContent = File.ReadAllLines(pathToSDFFile);

        //////File.Delete(pathToSDFFile);
        //
        //ADD Pose and scale to models
        //
        foreach (string modelName in m_ModelNames)
        {
            string absoluteModelPath = UpdaterUtility.ProjectFolder + @"/SimulationModels/" + modelName + "/OriginModels";
            //Create GameObject where everything will be attached
            GameObject modelParent = new GameObject(modelName);


            //List for all downloaded visuals
            List<string> visMeshList = new List<string>();
            if (absoluteModelPath != "")
            {
                visMeshList = UpdaterUtility.getFilePathsFBX(absoluteModelPath + @"/visual");
            }

            //List for all downloaded colliders
            List<string> colMeshList = new List<string>();
            if (absoluteModelPath != "")
            {
                colMeshList = UpdaterUtility.getFilePathsFBX(absoluteModelPath + @"/collision");
            }

            foreach (string name in visMeshList)
            {
                GameObject meshPrefab = null;
                string relativeModelPath = "Assets/SimulationModels/" + modelName + "/OriginModels/";
                // import Mesh
                meshPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(relativeModelPath + @"visual/" + name, typeof(Object));
                //StartCoroutine(importModelCoroutine(path, (result) => { meshPrefab = result; }));
                if (meshPrefab == null)
                {
                    Debug.Log("Could not import model!");
                    continue;
                }

                GameObject meshCopy = Instantiate(meshPrefab);

                meshCopy.tag = "RoboyPart";
                UpdaterUtility.attachCollider(meshCopy, relativeModelPath, name);

                var regex1 = new Regex(Regex.Escape("(Clone)"));
                meshCopy.name = regex1.Replace(meshCopy.name, "", 1);

                var regex2 = new Regex(Regex.Escape("VIS_"));
                meshCopy.name = regex2.Replace(meshCopy.name, "", 1);

                SelectableObject selectableObjectComponent = meshCopy.AddComponent<SelectableObject>();
                selectableObjectComponent.TargetedMaterial = Resources.Load("RoboyMaterials/TargetedMaterial") as Material;
                selectableObjectComponent.SelectedMaterial = Resources.Load("RoboyMaterials/SelectedMaterial") as Material;

                meshCopy.AddComponent<RoboyPart>();
                // Attach Model with mesh to parent GO
                meshCopy.transform.parent = modelParent.transform;
            }
            modelParent.tag = "Roboy";

            //Create Prefab of existing GO
            Object prefab = PrefabUtility.CreateEmptyPrefab("Assets/SimulationModels/" + modelName + "/" + modelName + ".prefab");
            PrefabUtility.ReplacePrefab(modelParent, prefab, ReplacePrefabOptions.ConnectToPrefab);
            //Destroy GO after prefab is created
            DestroyImmediate(modelParent);
        }
        m_ModelNames.Clear();
    }


    ///// <summary>
    ///// Tries to import the model in the given path. Does not work like intended cause of reasons.
    ///// </summary>
    ///// <param name="path"></param>
    ///// <param name="callback"></param>
    ///// <returns></returns>
    //private IEnumerator importModelCoroutine(string path, System.Action<GameObject> callback)
    //{
    //    // create a counter to limit the tries
    //    int modelImportCounter = 0;
    //    GameObject meshPrefab = null;
    //    // try to import the model 100 times
    //    while (modelImportCounter < 100)
    //    {
    //        meshPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(path, typeof(Object));
    //        modelImportCounter++;
    //        // if prefab is loaded then set the prefab to the given gameobject at the coroutine call
    //        if (meshPrefab != null)
    //        {
    //            if (callback != null) callback(meshPrefab);
    //            break;
    //        }
    //        yield return new WaitForSeconds(0.1f);
    //    }
    //}
}
