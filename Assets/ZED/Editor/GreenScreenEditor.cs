#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.IO;
[CustomEditor(typeof(GreenScreenManager))]
class GreenScreenManagerEditor : Editor
{
    private GreenScreenManager greenScreen;
    private GUILayoutOption[] optionsButton = { GUILayout.MaxWidth(100) };

    private GUILayoutOption[] optionsButtonBrowse = { GUILayout.MaxWidth(30) };

    private SerializedProperty keyColors;
    private SerializedProperty range;
    private SerializedProperty smoothness;
    private SerializedProperty whiteclip;
    private SerializedProperty blackclip;
    private SerializedProperty erosion;
    private SerializedProperty sigma;
    private SerializedProperty despill;

    public void OnEnable()
    {
        greenScreen = (GreenScreenManager)target;


        keyColors = serializedObject.FindProperty("keyColors");
        erosion = serializedObject.FindProperty("erosion");

        range = serializedObject.FindProperty("range");
        smoothness = serializedObject.FindProperty("smoothness");
        whiteclip = serializedObject.FindProperty("whiteClip");
        blackclip = serializedObject.FindProperty("blackClip");
        sigma = serializedObject.FindProperty("sigma_");
        despill = serializedObject.FindProperty("spill");


    }
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        greenScreen = (GreenScreenManager)target;
        key_colors();
        GUILayout.Space(20);
        EditorGUILayout.BeginHorizontal();
        greenScreen.pathFileConfig = EditorGUILayout.TextField("Save Config", greenScreen.pathFileConfig);
        if (GUILayout.Button("...", optionsButtonBrowse))
        {
            greenScreen.pathFileConfig = EditorUtility.OpenFilePanel("Load save file", "", "*");
            serializedObject.ApplyModifiedProperties();
        }
        EditorGUILayout.EndHorizontal(); EditorGUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();

        if (GUILayout.Button("Save"))
        {
            greenScreen.SaveChromaKeys();
        }
        GUI.enabled = File.Exists(greenScreen.pathFileConfig);
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
        //GUILayout.Label("Chroma Key", EditorStyles.boldLabel);
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

        keyColors.colorValue = EditorGUILayout.ColorField("Color", keyColors.colorValue);

        range.floatValue = EditorGUILayout.Slider("Range", range.floatValue, 0.0f, 1.0f);
        smoothness.floatValue = EditorGUILayout.Slider("Smoothness", smoothness.floatValue, 0, 1.0f);

        EditorGUILayout.Space();
        GUILayout.Label("Foreground", EditorStyles.boldLabel);
        whiteclip.floatValue = EditorGUILayout.Slider("Clip White", whiteclip.floatValue, 0.0f, 1.0f);
        blackclip.floatValue = EditorGUILayout.Slider("Clip Black", blackclip.floatValue, 0.0f, 1.0f);

        erosion.intValue = EditorGUILayout.IntSlider("Erosion", erosion.intValue, 0, 5);
        sigma.floatValue = EditorGUILayout.Slider("Edges Softness", sigma.floatValue, 0.1f, 2.0f);

        despill.floatValue = EditorGUILayout.Slider("Despill", despill.floatValue, 0f, 1f);

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