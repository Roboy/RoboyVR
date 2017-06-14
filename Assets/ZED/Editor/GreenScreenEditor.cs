#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.IO;
[CustomEditor(typeof(GreenScreenManager))]
class GreenScreenManagerEditor : Editor
{
    private const uint max_keys = 5;

    private GreenScreenManager greenScreen;
    private GUILayoutOption[] optionsButton = { GUILayout.MaxWidth(100) };

    private GUILayoutOption[] optionsButtonBrowse = { GUILayout.MaxWidth(30) };


    public void OnEnable()
    {
        greenScreen = (GreenScreenManager)target;
        //greenScreen.SetDefaultValues();

    }
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        key_colors();
        GUILayout.Space(20);
        EditorGUILayout.BeginHorizontal();
        greenScreen.pathFileShader = EditorGUILayout.TextField("Save Config", greenScreen.pathFileShader);
        if (GUILayout.Button("...", optionsButtonBrowse))
        {
            greenScreen.pathFileShader = EditorUtility.OpenFilePanel("Load save file", "", "*");
            serializedObject.ApplyModifiedProperties();
        }
        EditorGUILayout.EndHorizontal(); EditorGUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();

        if (GUILayout.Button("Save"))
        {
            greenScreen.SaveChromaKeys();
        }
        GUI.enabled = File.Exists(greenScreen.pathFileShader);
        if (GUILayout.Button("Load"))
        {
            greenScreen.LoadChromaKeys();
        }
        GUI.enabled = true;

        EditorGUILayout.EndHorizontal();
        GUILayout.Space(20);
        if (GUILayout.Button("Camera Control"))
        {
            EditorWindow.GetWindow(typeof(ZEDEditor), false, "ZED Camera").Show();
        }
        
        serializedObject.ApplyModifiedProperties();
    }

    void key_colors()
    {
        EditorGUILayout.Space();
        EditorGUI.BeginChangeCheck();
        greenScreen.canal = (GreenScreenManager.CANAL)EditorGUILayout.EnumPopup("View", greenScreen.canal);
        EditorGUILayout.Space();

        if (EditorGUI.EndChangeCheck())
        {
            greenScreen.UpdateCanal();
        }
        EditorGUI.BeginChangeCheck();

        EditorGUILayout.BeginHorizontal();


        EditorGUILayout.EndHorizontal();

        GUILayout.Label("Screen", EditorStyles.boldLabel);

        serializedObject.FindProperty("keyColors").colorValue = EditorGUILayout.ColorField("Color", serializedObject.FindProperty("keyColors").colorValue);

        serializedObject.FindProperty("range").floatValue = EditorGUILayout.Slider("Range", serializedObject.FindProperty("range").floatValue, 0.0f, 1.0f);
        serializedObject.FindProperty("smoothness").floatValue = EditorGUILayout.Slider("Smoothness", serializedObject.FindProperty("smoothness").floatValue, 0, 1.0f);

        EditorGUILayout.Space();
        GUILayout.Label("Foreground", EditorStyles.boldLabel);
        serializedObject.FindProperty("whiteClip").floatValue = EditorGUILayout.Slider("Clip White", serializedObject.FindProperty("whiteClip").floatValue, 0.0f, 1.0f);
        serializedObject.FindProperty("blackClip").floatValue = EditorGUILayout.Slider("Clip Black", serializedObject.FindProperty("blackClip").floatValue, 0.0f, 1.0f);

        serializedObject.FindProperty("erosion").intValue = EditorGUILayout.IntSlider("Erosion", serializedObject.FindProperty("erosion").intValue, 0, 5);
        serializedObject.FindProperty("sigma_").floatValue = EditorGUILayout.Slider("Edges Softness", serializedObject.FindProperty("sigma_").floatValue, 0.1f, 2.0f);

        serializedObject.FindProperty("spill").floatValue = EditorGUILayout.Slider("Despill", serializedObject.FindProperty("spill").floatValue, 0f, 1f);

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Default", optionsButton))
        {
            greenScreen.SetDefaultValues();
            Repaint();
        }
        GUILayout.EndHorizontal();


        if (EditorGUI.EndChangeCheck())
        {
            greenScreen.UpdateShader();
        }


    }
}
#endif