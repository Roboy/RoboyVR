using System;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// Custom editor script to be able to call functions from MeshUpdater at edit time through buttons.
/// </summary>
[CustomEditor(typeof(MeshUpdater))]
class MeshUpdaterEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();

        MeshUpdater meshUpdater = (MeshUpdater)target;

        // Show the blender path if it is set
        if (meshUpdater.CurrentState >= MeshUpdater.State.BlenderPathSet )
        {   
            //Disable GUI so it can't be edited in UnityEditor
            GUI.enabled = false;
            EditorGUILayout.TextField("Blender Path: ", meshUpdater.PathToBlender);
            GUI.enabled = true;
        }

        // Button to select the blender.exe
        if (GUILayout.Button("Open Blender directory"))
        {
            // open the ProgramFiles directory of this system
            string blenderpath = EditorUtility.OpenFilePanel("Select blender.exe", Environment.GetEnvironmentVariable("ProgramFiles"), "exe");
            // If the user did not select and exe then show dialog
            if (string.IsNullOrEmpty(blenderpath))
            {
                EditorUtility.DisplayDialog("Select blender.exe", "You must select the blender.exe!", "OK");
            }
            // Else update blender path
            else
            {
                meshUpdater.PathToBlender = blenderpath;
            }
        }

        // Do not show Scan, Download and Create Prefab button if blender path is not set
        if (meshUpdater.CurrentState < MeshUpdater.State.BlenderPathSet)
            return;

        if (GUILayout.Button("Scan"))
        {
            meshUpdater.Scan();
        }

        // stop here if meshupdater did not scan yet
        if (meshUpdater.CurrentState < MeshUpdater.State.Scanned)
            return;

        // ? THIS WON'T RESET WHEN DICTIONARY CLEARS ?
        var keys = new List<string>(meshUpdater.ModelChoiceDictionary.Keys);
        // show checkboxes for each model entry and update their values
        foreach (string modelEntryKey in keys)
        {
            meshUpdater.ModelChoiceDictionary[modelEntryKey] = EditorGUILayout.Toggle(modelEntryKey, meshUpdater.ModelChoiceDictionary[modelEntryKey]);
        }

        // downloads models and converts them to .fbx (models will also be imported into unity)
        if (GUILayout.Button("Download"))
        {
            meshUpdater.UpdateModels();
        }

        // don't show "Create Prefab" before models are imported
        if (meshUpdater.CurrentState < MeshUpdater.State.Downloaded)
            return;

        // create prefab
        if (GUILayout.Button("Create Prefab"))
        {
            meshUpdater.CreatePrefab();
        }

    }
}