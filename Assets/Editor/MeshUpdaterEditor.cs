using System;
using UnityEngine;
using UnityEditor;

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
        if (meshUpdater.PathToBlenderSet)
        {
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

        // Do not show Scan and Update button if blender path is not set
        if (!meshUpdater.PathToBlenderSet)
            return;

        if (GUILayout.Button("Scan"))
        {
            meshUpdater.Scan();
        }
        if (GUILayout.Button("Update"))
        {
            meshUpdater.UpdateModels();
        }
            
    }
}