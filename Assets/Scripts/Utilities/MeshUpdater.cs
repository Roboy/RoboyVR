using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;

[ExecuteInEditMode]
public class MeshUpdater : MonoBehaviour {

    /// <summary>
    /// State enum to track the current state of the mesh updater
    /// </summary>
    public enum State
    {
        None = 0,
        Initialized = 1,
        BlenderPathSet = 2,
        Scanned = 3,
        Downloaded = 4
    }

    /// <summary>
    /// Github repository of the roboy models.
    /// </summary>
    public string Github_Repository = @"https://github.com/Roboy/roboy_models/";

    /// <summary>
    /// Path to blender.exe. Is set via the user via a file selection through the file explorer.
    /// </summary>
    [HideInInspector]
    public string PathToBlender {
        get { return m_PathToBlender; }
        set {
                m_PathToBlender =  value;
                m_CurrentState = (State) Mathf.Max((int)State.BlenderPathSet, (int)m_CurrentState);
        }
    }
    /// <summary>
    /// Public property of the URL Dic for the editor script
    /// </summary>
    public Dictionary<string, string> URLDictionary { get { return m_URLDictionary; } }

    /// <summary>
    /// Dictionary to store the users choice whether he wants to update the model or not
    /// </summary>
    public Dictionary<string, bool> ModelChoiceDictionary = new Dictionary<string, bool>();

    /// <summary>
    /// Public property for the editor script
    /// </summary>
    public State CurrentState { get { return m_CurrentState; } }

    /// <summary>
    /// Current state of the meshupdater
    /// </summary>
    private State m_CurrentState = State.None;

    /// <summary>
    /// Private variable for the blender path to encapsulate the get and set in a property instead of a function.
    /// </summary>
    private string m_PathToBlender;

    /// <summary>
    /// This should be the path to the "MeshDownloader". It is located in the ExternalTools directory.
    /// </summary>
    private string m_PathToDownloadScript;

    /// <summary>
    /// This should be the path to the "MeshScanner". It is located in the ExternalTools directory.
    /// </summary>
    private string m_PathToScanScript;

    /// <summary>
    /// Cached variable of the projects assets directory.
    /// </summary>
    private string m_ProjectFolder;

    /// <summary>
    /// Stores all model "Titles + URLs"
    /// </summary>
    private Dictionary<string, string> m_URLDictionary = new Dictionary<string, string>();

    /// <summary>
    /// Temp ModelName 
    /// </summary>
    private List<string> m_ModelNames = new List<string>();

    // Use this for initialization
    void Awake () {
        Initialize();
    }

    /// <summary>
    /// Initializes the paths of the python scripts.
    /// </summary>
    public void Initialize()
    {
        m_ProjectFolder = Application.dataPath;
        m_PathToDownloadScript = m_ProjectFolder + @"/ExternalTools/ModelDownloader.py";
        m_PathToScanScript = m_ProjectFolder + @"/ExternalTools/ModelScanner.py";

        showWarnings();
    }

    /// <summary>
    /// Scans the repository for the roboy models and stores them in a dictionary.
    /// </summary>
    public void Scan()
    {
        string[] scanArguments = { "python", m_PathToScanScript, Github_Repository };
        CommandlineUtility.RunCommandLine(scanArguments);
        // to do whether scan file exists and is right
        // check whether file exists
        string pathToScanFile = m_ProjectFolder + @"/tempModelURLs.txt";
        if (!File.Exists(pathToScanFile))
        {
            Debug.LogWarning("Scan file not found! Check whether it exists or if python script is working!");
            return;
        }
        // get file content of format title;url
        string[] scanContent = File.ReadAllLines(pathToScanFile);
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
        //Debug.Log("before clear: " + m_URLDictionary.Count);
        m_URLDictionary.Clear();
        //Debug.Log("after clear: " + m_URLDictionary.Count);
        m_URLDictionary = tempURLDic;
        ModelChoiceDictionary.Clear();
        foreach (var urlDicEntry in m_URLDictionary)
        {
            ModelChoiceDictionary.Add(urlDicEntry.Key, false);
        }
        m_CurrentState = State.Scanned;
    }

    /// <summary>
    /// Downloads the models from the scan dictionary which were selected by the user.
    /// </summary>
    public void UpdateModels()
    {
        string pathToOriginModels = m_ProjectFolder + @"/SimulationModels/";
        //var processInfo = new ProcessStartInfo("cmd.exe", "/C" + "start \"\" \"" + m_PathToBlender + "\" -P \"" + m_PathToDownloadScript + "\" \"" + pathToMeshes + "\" \"" + m_pathToProjectModels + "\" \"\"");
        //UnityEngine.Debug.Log("Run not implemented yet!");
        // get a list of all entries which the user wants to update
        List<KeyValuePair<string, bool>> tempURLList = ModelChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {
            string[] scanArguments = { "python", m_PathToScanScript, m_URLDictionary[urlEntry.Key] };
            CommandlineUtility.RunCommandLine(scanArguments);
            //Debug.Log(m_URLDictionary[urlEntry.Key]);

            string pathToScanFile = m_ProjectFolder + @"/tempModelURLs.txt";
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
                //start modeldownloader.py
                string[] updateArguments = { "start \"\" \""+ m_PathToBlender + "\" -P", m_PathToDownloadScript, titleURL[1] + @"/", m_ProjectFolder + @"/SimulationModels/" + urlEntry.Key, "" };
                CommandlineUtility.RunCommandLine(updateArguments);

                if (!m_ModelNames.Contains(urlEntry.Key)) {
                    m_ModelNames.Add(urlEntry.Key);                               
                }
            }
            m_CurrentState = State.Downloaded;
        }  
    }

    /// <summary>
    /// Creates prefabs for every model which were downloaded. 
    /// </summary>
    public void CreatePrefab() {
        foreach (string modelName in m_ModelNames)
        {   
            string pathToOriginModels = m_ProjectFolder + @"/SimulationModels/" + modelName;
            //Create GameObject where everything will be attached
            GameObject modelParent = new GameObject(modelName);
            List<string> meshList = new List<string>();
            if (pathToOriginModels != "")
            {
                meshList = getFilePathsFBX(pathToOriginModels + "/OriginModels");
            }

            foreach (string name in meshList)
            {
                GameObject meshPrefab = null;
                string path = "Assets/SimulationModels/" + modelName + "/OriginModels/" + name;
                // import Mesh
                StartCoroutine(importModelCoroutine(path, (result) => { meshPrefab = result; }));
                if (meshPrefab == null)
                {
                    Debug.Log("Could not import model!");
                    continue;
                }

                GameObject meshCopy = Instantiate(meshPrefab);
                // Attach Model with mesh to parent GO
                meshCopy.transform.parent = modelParent.transform;
            }
            //Create Prefab of existing GO
            Object prefab = PrefabUtility.CreateEmptyPrefab("Assets/SimulationModels/" + modelName + "/" + modelName + ".prefab");
            PrefabUtility.ReplacePrefab(modelParent, prefab, ReplacePrefabOptions.ConnectToPrefab);
            //Destroy GO after prefab is created
            DestroyImmediate(modelParent);
        }
        m_ModelNames.Clear();
    }

    /// <summary>
    /// Tries to import the model in the given path. Does not work like intended cause of reasons.
    /// </summary>
    /// <param name="path"></param>
    /// <param name="callback"></param>
    /// <returns></returns>
    private IEnumerator importModelCoroutine(string path, System.Action<GameObject> callback)
    {
        // create a counter to limit the tries
        int modelImportCounter = 0;
        GameObject meshPrefab = null;
        // try to import the model 100 times
        while (modelImportCounter < 100)
        {
            meshPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(path, typeof(Object));
            modelImportCounter++;
            // if prefab is loaded then set the prefab to the given gameobject at the coroutine call
            if (meshPrefab != null)
            {
                if (callback != null) callback(meshPrefab);
                break;
            }
            yield return new WaitForSeconds(0.1f);
        }
    }

    /// <summary>
    /// Returns fbx file paths in the given directory.
    /// </summary>
    /// <param name="sDir">The directory you want to search.</param>
    /// <returns>List of all fbx file paths.</returns>
    private List<string> getFilePathsFBX(string sDir)
    {
        List<string> files = new List<string>();
        {
            foreach (string f in Directory.GetFiles(sDir))
            {   
                if(Path.GetExtension(f) == ".fbx")
                    files.Add(Path.GetFileName(f));
            }
        }
        return files;
    }

    /// <summary>
    /// Shows warnings for each python script.
    /// </summary>
    private void showWarnings()
    {
        if (File.Exists(m_PathToDownloadScript))
        {
            Debug.Log("Download script found!");
        }
        else
        {
            Debug.LogWarning("Download script not found!");
        }

        if (File.Exists(m_PathToScanScript))
        {
            Debug.Log("Scan script found!");
        }
        else
        {
            Debug.LogWarning("Scan script not found!");
        }
    }
}
