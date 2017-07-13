using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;

[ExecuteInEditMode]
public class WorldUpdater : MonoBehaviour
{
    struct ModelTransformation
    {
        public string worldname;
        public string name;
        public Vector3 position;
        public Vector3 rotation;
        public Vector3 scale;
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

    public void Magic() {
        List<KeyValuePair<string, bool>> tempURLList = WorldChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {
            for (int i = 0; i <= 3; i++) {
                ModelTransformation testModel = new ModelTransformation();
                testModel.worldname = urlEntry.Key;
                testModel.name = "construction_cone";
                testModel.position = new Vector3(Random.Range(-10.0f, 10.0f), 0, Random.Range(-10.0f, 10.0f));
                testModel.rotation = new Vector3(0, 0, 0);
                testModel.scale = new Vector3(1, 1, 1);
                modellist.Add(testModel);
            }
            WorldsCurrentState = UpdaterUtility.State.Downloaded;
        }
    }


    public void CreateWorld()
    {
        List<string> names = new List<string>();
        foreach (ModelTransformation model in modellist)
        {
            if (!names.Contains(model.name))
            {
                names.Add(model.name);
            }
        }
        foreach (string name in names)
        {

            //replace "tree" with "raw" in URL
            var regex = new Regex(Regex.Escape("tree"));
            m_PathToModelsFolder = regex.Replace(m_PathToModelsFolder, "raw", 1);
            //start modeldownloader.py for visual
            string[] updateArgumentsWorld = { "start \"\" \"" + UpdaterUtility.PathToBlender + "\" -P", UpdaterUtility.PathToDownloadScript, m_PathToModelsFolder + @"/" + name + @"/meshes/", UpdaterUtility.ProjectFolder + @"/SimulationWorlds/Models/" + name, "" };
            CommandlineUtility.RunCommandLine(updateArgumentsWorld);
        }

        List<KeyValuePair<string, bool>> tempURLList = WorldChoiceDictionary.Where(entry => entry.Value == true).ToList();
        foreach (var urlEntry in tempURLList)
        {
            GameObject worldParent = new GameObject(urlEntry.Key);
            foreach (ModelTransformation model in modellist)
            {
                string absoluteWorldPath = UpdaterUtility.ProjectFolder + @"/SimulationWorlds/" + model.name;
                //Create GameObject where everything will be attached
                if (model.worldname == urlEntry.Key) {
                    GameObject modelParent = new GameObject(model.name);
                }                
                
            }
        }
    }
}
