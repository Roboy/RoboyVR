using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ZEDManager)), CanEditMultipleObjects]
public class ZEDManagerInspector : Editor {

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        GUILayout.Space(20);
        if (GUILayout.Button("Camera Control"))
        {
            EditorWindow.GetWindow(typeof(ZEDEditor), false, "ZED Camera").Show();
        }

    }
}
