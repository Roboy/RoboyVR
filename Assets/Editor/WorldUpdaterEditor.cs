using System;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// Custom editor script to be able to call functions from WorldUpdater at edit time through buttons.
/// </summary>
[CustomEditor(typeof(WorldUpdater))]
class WorldUpdaterEditor : Editor
{ 


    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();

        WorldUpdater worldUpdater = (WorldUpdater)target;

        // Show the blender path if it is set
        if (worldUpdater.WorldsCurrentState >= UpdaterUtility.State.BlenderPathSet)
        {
            //Disable GUI so it can't be edited in UnityEditor
            GUI.enabled = false;
            EditorGUILayout.TextField("Blender Path: ", UpdaterUtility.PathToBlender);
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
                UpdaterUtility.PathToBlender = blenderpath;
                worldUpdater.WorldsCurrentState = (UpdaterUtility.State)Mathf.Max((int)UpdaterUtility.State.BlenderPathSet, (int)worldUpdater.WorldsCurrentState);
            }
        }

        // Do not show Scan, Download and Create World button if blender path is not set
        if (worldUpdater.WorldsCurrentState < UpdaterUtility.State.BlenderPathSet)
            return;

        if (GUILayout.Button("Scan"))
        {
            worldUpdater.Scan();
        }

        // stop here if WorldUpdater did not scan yet
        if (worldUpdater.WorldsCurrentState < UpdaterUtility.State.Scanned)
            return;

        // ? THIS WON'T RESET WHEN DICTIONARY CLEARS ?
        var keys = new List<string>(worldUpdater.WorldChoiceDictionary.Keys);
        // show checkboxes for each model entry and update their values
        foreach (string modelEntryKey in keys)
        {
            worldUpdater.WorldChoiceDictionary[modelEntryKey] = EditorGUILayout.Toggle(modelEntryKey, worldUpdater.WorldChoiceDictionary[modelEntryKey]);
        }

        
        if (GUILayout.Button("Download .world files"))
        {   
            //Download .world file
            worldUpdater.LoadWorlds();
            //Use .world file to download models
            worldUpdater.Magic();
        }

        // don't show "Create Prefab" before models are imported
        if (worldUpdater.WorldsCurrentState < UpdaterUtility.State.Downloaded)
            return;

        // create prefab
        if (GUILayout.Button("Create World"))
        {
            worldUpdater.CreateWorld();
        }

    }

}
