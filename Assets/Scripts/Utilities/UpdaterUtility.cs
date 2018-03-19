using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using UnityEditor;

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
    /// This should be the path to the "SDFreader". It is located in the ExternalTools directory.
    /// </summary>
    public static string PathToSDFreader;

    /// <summary>
    /// This should be the path to the "MeshScanner". It is located in the ExternalTools directory.
    /// </summary>
    public static string PathToWorldReader;

    /// <summary>
    /// Cached variable of the projects assets directory.
    /// </summary>
    public static string ProjectFolder;


    /// <summary>
    /// Returns fbx file paths in the given directory.
    /// </summary>
    /// <param name="sDir">The directory you want to search.</param>
    /// <returns>List of all fbx file paths.</returns>
    public static List<string> getFilePathsFBX(string sDir)
    {
        List<string> files = new List<string>();
        {
            foreach (string f in Directory.GetFiles(sDir))
            {
                if (Path.GetExtension(f) == ".fbx")
                    files.Add(Path.GetFileName(f));
            }
        }
        return files;
    }

    /// <summary>
    /// Attaches a collider to the given gameObject.
    /// </summary>
    /// <param name="meshGO">The gameObject you want to attach the colliders on.</param>
    /// <param name="path">The path of the parent object in the Origin folder.</param>
    /// <param name="modelName">The actual name of the visual model.</param>
    public static void attachCollider(GameObject meshGO, string path, string modelName)
    {
        // replace name from visual to collision
        modelName = modelName.Replace("VIS_", "COL_");
        // get the object which serves as mesh collider
        GameObject colliderPrefab = (GameObject)AssetDatabase.LoadAssetAtPath(path + @"collision/" + modelName, typeof(Object));

        if (colliderPrefab == null)
        {
            Debug.Log("[UpdaterUtility] Did not find a collider object for mesh: " + modelName);
            return;
        }
        // go through each mesh filter and add all mesh references as mesh colliders to the gameObject
        List<MeshFilter> collRenderers = colliderPrefab.GetComponentsInChildren<MeshFilter>().ToList();

        foreach (MeshFilter collRenderer in collRenderers)
        {   
            MeshCollider meshCollider = meshGO.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = collRenderer.sharedMesh;
        }
    }


    /// <summary>
    /// Shows warnings for each python script.
    /// </summary>
    public static void showWarnings()
    {
        if (File.Exists(PathToDownloadScript))
        {
            Debug.Log("[UpdaterUtility] Download script found!");
        }
        else
        {
            Debug.LogWarning("[UpdaterUtility] Download script not found!");
        }

        if (File.Exists(PathToScanScript))
        {
            Debug.Log("[UpdaterUtility] Scan script found!");
        }
        else
        {
            Debug.LogWarning("[UpdaterUtility] Scan script not found!");
        }

        if (File.Exists(PathToSDFreader))
        {
            Debug.Log("[UpdaterUtility] SDF_Reader script found!");
        }
        else
        {
            Debug.LogWarning("[UpdaterUtility] SDF_Reader script not found!");
        }

        if (File.Exists(PathToWorldReader))
        {
            Debug.Log("[UpdaterUtility] World_Reader script found!");
        }
        else
        {
            Debug.LogWarning("[UpdaterUtility] World_Reader script not found!");
        }


    }
}
