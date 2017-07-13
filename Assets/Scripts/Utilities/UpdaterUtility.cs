using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class UpdaterUtility : MonoBehaviour {



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
    /// Path to blender.exe. Is set via the user via a file selection through the file explorer.
    /// </summary>
    [HideInInspector]
    public static string PathToBlender
    {
        get { return m_PathToBlender; }
        set
        {
            m_PathToBlender = value;
        }
    }


    /// <summary>
    /// Private variable for the blender path to encapsulate the get and set in a property instead of a function.
    /// </summary>
    private static string m_PathToBlender;


    /// <summary>
    /// This should be the path to the "MeshDownloader". It is located in the ExternalTools directory.
    /// </summary>
    public static string PathToDownloadScript;

    /// <summary>
    /// This should be the path to the "MeshScanner". It is located in the ExternalTools directory.
    /// </summary>
    public static string PathToScanScript;

    /// <summary>
    /// Cached variable of the projects assets directory.
    /// </summary>
    public static string ProjectFolder;


    /// <summary>
    /// Shows warnings for each python script.
    /// </summary>
    public static void showWarnings()
    {
        if (File.Exists(PathToDownloadScript))
        {
            Debug.Log("Download script found!");
        }
        else
        {
            Debug.LogWarning("Download script not found!");
        }

        if (File.Exists(PathToScanScript))
        {
            Debug.Log("Scan script found!");
        }
        else
        {
            Debug.LogWarning("Scan script not found!");
        }
    }
}
