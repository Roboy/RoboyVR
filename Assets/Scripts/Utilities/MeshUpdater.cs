using UnityEngine;
using System.IO;

[ExecuteInEditMode]
public class MeshUpdater : MonoBehaviour {

    public string Github_Repository = @"https://github.com/Roboy/roboy_models/";

    /// <summary>
    /// Path to blender.exe. Is set via the user via a file selection through the file explorer.
    /// </summary>
    [HideInInspector]
    public string PathToBlender {
        get { return m_PathToBlender; }
        set { m_PathToBlender =  value; PathToBlenderSet = true; }
    }

    /// <summary>
    /// Boolean to check whether the blender path is set.
    /// </summary>
    [HideInInspector]
    public bool PathToBlenderSet = false;

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
    /// Calls the python scan script through a commandline.
    /// </summary>
    public void Scan()
    {
        string[] scanArguments = { "python", m_PathToScanScript, Github_Repository };
        CommandlineUtility.RunCommandLine(scanArguments);
    }

    public void UpdateModels()
    {
        //var processInfo = new ProcessStartInfo("cmd.exe", "/C" + "start \"\" \"" + m_PathToBlender + "\" -P \"" + m_PathToDownloadScript + "\" \"" + pathToMeshes + "\" \"" + m_pathToProjectModels + "\" \"\"");
        UnityEngine.Debug.Log("Run not implemented yet!");
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
