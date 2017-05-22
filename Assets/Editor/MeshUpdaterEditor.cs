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
        if (GUILayout.Button("Scan"))
        {
            MeshUpdater meshUpdate = (MeshUpdater)target;
            //MeshUpdater a = new MeshUpdater();
            // \"\" = ""
            meshUpdate.RunCMD("python \""+ meshUpdate.pathToScanScript+"\" \""+meshUpdate.pathToModels +"\"");
        }
        if (GUILayout.Button("Run"))
        {
            MeshUpdater meshUpdate = (MeshUpdater)target;
            //MeshUpdater a = new MeshUpdater();
            // \"\" = ""
            meshUpdate.RunCMD("start \"\" \""+ meshUpdate.pathToBlender + "\" -P \""+ meshUpdate.pathToScript + "\" \""+ meshUpdate.pathToMeshes+ "\" \"\"");
        }
            
    }
}