using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MeshUpdater))]
class MeshUpdaterEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        //GUILayout.TextField("text");
        if (GUILayout.Button("Run"))
        {
            MeshUpdater a = new MeshUpdater();
            a.MeshUpdate(a.pathToScript, a.pathToBlender+a.pathToMeshes);
        }
            
    }
}