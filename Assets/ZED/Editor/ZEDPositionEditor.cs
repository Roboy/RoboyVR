using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ZEDPositionManager))]
public class ZEDPositionEditor : Editor
{
    private ZEDPositionManager positionManager;

    public void OnEnable()
    {
        positionManager = (ZEDPositionManager)target;

    }

    public override void OnInspectorGUI()
    {
        GUILayout.Space(5);
        EditorGUILayout.BeginHorizontal();

        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Save Camera Offset"))
        {
            positionManager.SaveZEDPos();
        }
        if (GUILayout.Button("Load Camera Offset"))
        {
            positionManager.LoadZEDPos();
        }
        EditorGUILayout.EndHorizontal();
    }
}
