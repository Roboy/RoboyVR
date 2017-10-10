using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ZEDGarbageMatte))]
class GarbageMatteEditor : Editor
{
    private ZEDGarbageMatte matte;
    private GUILayoutOption[] optionsButtonBrowse = { GUILayout.MaxWidth(30) };

    SerializedProperty garbageMattePath;
    SerializedProperty loadAtStart;
    SerializedProperty editMode;

    private static GUIStyle ToggleButtonStyleNormal = null;
    private static GUIStyle ToggleButtonStyleToggled = null;
    public void OnEnable()
    {
        matte = (ZEDGarbageMatte)target;

        editMode = serializedObject.FindProperty("editMode");
        garbageMattePath = serializedObject.FindProperty("garbageMattePath");
        loadAtStart = serializedObject.FindProperty("loadAtStart");
    }

    public override void OnInspectorGUI()
    {
        if (ToggleButtonStyleNormal == null)
        {
            ToggleButtonStyleNormal = "Button";
            ToggleButtonStyleToggled = new GUIStyle(ToggleButtonStyleNormal);
            ToggleButtonStyleToggled.normal.background = ToggleButtonStyleToggled.active.background;
        }

        matte = (ZEDGarbageMatte)target;

        serializedObject.Update();
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Edit", editMode.boolValue ? ToggleButtonStyleToggled : ToggleButtonStyleNormal))
        {
            editMode.boolValue = !editMode.boolValue;
            if (editMode.boolValue)
            {
                matte.EnterEditMode();
            }

        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Remove last point"))
        {
            matte.RemoveLastPoint();
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Apply Garbage Matte"))
        {
            matte.ApplyGarbageMatte();
        }
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
       

        garbageMattePath.stringValue = EditorGUILayout.TextField("Garbage Matte File", garbageMattePath.stringValue);

        if (GUILayout.Button("...", optionsButtonBrowse))
        {
            matte.garbageMattePath = EditorUtility.OpenFilePanel("Load save file", "", "*");
            serializedObject.ApplyModifiedProperties();
        }


       
         loadAtStart.boolValue = EditorGUILayout.Toggle(GUIContent.none, loadAtStart.boolValue, GUILayout.MaxWidth(20));

        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Save"))
        {
            matte.Save();
        }
        GUI.enabled = System.IO.File.Exists(matte.garbageMattePath);
        if (GUILayout.Button("Load"))
        {
            matte.Load();
        }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();
        if (GUILayout.Button("Reset"))
        {
            matte.ResetPoints();
        }
        serializedObject.ApplyModifiedProperties();
    }
}

