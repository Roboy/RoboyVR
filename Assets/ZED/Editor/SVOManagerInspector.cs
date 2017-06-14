using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(SVOManager)), CanEditMultipleObjects]
public class SVOManagerInspector : Editor
{


    private bool current_recordValue = false;
    private bool current_readValue = false;

    Rect drop_area;

    private GUILayoutOption[] optionsButtonBrowse = { GUILayout.MaxWidth(30) };

    string[] filters = { "Svo files", "svo" };
    private SVOManager obj;
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        obj = (SVOManager)target;
        EditorGUI.BeginChangeCheck();
        DrawDefaultInspector();
        EditorGUILayout.BeginHorizontal();
        obj.videoFile = EditorGUILayout.TextField("SVO Path", obj.videoFile);
        if (GUILayout.Button("...", optionsButtonBrowse))
        {
            obj.videoFile = EditorUtility.OpenFilePanelWithFilters("Load SVO", "", filters);
            serializedObject.ApplyModifiedProperties();
        }
        EditorGUILayout.EndHorizontal();
        if (drop_area.width != EditorGUIUtility.currentViewWidth || drop_area.height != Screen.height)
        {
            drop_area = new Rect(0, 0, EditorGUIUtility.currentViewWidth, Screen.height);
        }
        if (EditorGUI.EndChangeCheck())
        {
            CheckChange();
        }
        //serializedObject.ApplyModifiedProperties();
        DropAreaGUI();

        serializedObject.ApplyModifiedProperties();

    }

    private void CheckChange()
    {
        if (obj.loop && obj.record)
        {
            obj.loop = false;
        }
        if (obj.read && (current_readValue != obj.read))
        {
            obj.record = false;
            current_recordValue = false;
            current_readValue = obj.read;
        }
        if (!obj.read && (current_readValue != obj.read))
        {
            obj.loop = false;
        }
        if (obj.record && (current_recordValue != obj.record))
        {
            obj.read = false;
            current_readValue = false;
            obj.loop = false; 
            current_recordValue = obj.record;
        }

    }



    public void DropAreaGUI()
    {
        Event evt = Event.current;
        SVOManager obj = (SVOManager)target;
        switch (evt.type)
        {
            case EventType.DragUpdated:
            case EventType.DragPerform:
                if (!drop_area.Contains(evt.mousePosition))
                    return;
                DragAndDrop.visualMode = DragAndDropVisualMode.Copy;
                if (evt.type == EventType.DragPerform)
                {
                    DragAndDrop.AcceptDrag();
                    foreach (string dragged_object in DragAndDrop.paths)
                    {
                        obj.videoFile = dragged_object;
                    }
                }
                break;
        }
    }
}