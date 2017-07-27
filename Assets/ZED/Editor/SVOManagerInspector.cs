using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(SVOManager)), CanEditMultipleObjects]
public class SVOManagerInspector : Editor
{
    private bool current_recordValue = false;
    private bool current_readValue = false;

    private SerializedProperty pause;
    private SerializedProperty record;
    private SerializedProperty read;
    private SerializedProperty loop;
    private SerializedProperty videoFile;
    private SerializedProperty currentFrame;
    private SerializedProperty numberFrameMax;

    Rect drop_area;

    private GUILayoutOption[] optionsButtonBrowse = { GUILayout.MaxWidth(30) };
    string pauseText = "Pause";

    string[] filters = { "Svo files", "svo" };
    private SVOManager obj;
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        obj = (SVOManager)target;
        EditorGUI.BeginChangeCheck();
        DrawDefaultInspector();
        EditorGUILayout.BeginHorizontal();
        videoFile.stringValue = EditorGUILayout.TextField("SVO Path", videoFile.stringValue);
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
        EditorGUI.BeginChangeCheck();

        GUI.enabled = (obj.NumberFrameMax > 0);
        currentFrame.intValue = EditorGUILayout.IntSlider("Frame ", currentFrame.intValue, 0, numberFrameMax.intValue);
        if (EditorGUI.EndChangeCheck())
        {
            if (sl.ZEDCamera.GetInstance() != null)
            {
                sl.ZEDCamera.GetInstance().SetSVOPosition(currentFrame.intValue);
                if (pause.boolValue)
                {
                    sl.ZEDCamera.GetInstance().Grab();
                    sl.ZEDCamera.GetInstance().UpdateTextures();
                }
            }
        }
        GUI.enabled = true;

        GUI.enabled = sl.ZEDCamera.GetInstance() != null && sl.ZEDCamera.GetInstance().CameraIsReady;
        pauseText = pause.boolValue ? "Resume" : "Pause";
        if (GUILayout.Button(pauseText))
        {
            pause.boolValue = !pause.boolValue;
            ZEDUpdater.GetInstance().SetPauseThread(pause.boolValue);
        }
        GUI.enabled = true;
        DropAreaGUI();

        serializedObject.ApplyModifiedProperties();

    }

    private void OnEnable()
    {
        pause = serializedObject.FindProperty("pause");
        record = serializedObject.FindProperty("record");
        read = serializedObject.FindProperty("read");
        loop = serializedObject.FindProperty("loop");

        videoFile = serializedObject.FindProperty("videoFile");
        currentFrame = serializedObject.FindProperty("currentFrame");
        numberFrameMax = serializedObject.FindProperty("numberFrameMax");
    }

    private void CheckChange()
    {
        if (loop.boolValue && record.boolValue)
        {
            loop.boolValue = false;
        }
        if (read.boolValue && (current_readValue != read.boolValue))
        {
            record.boolValue = false;
            current_recordValue = false;
            current_readValue = read.boolValue;
        }
        if (!read.boolValue && (current_readValue != read.boolValue))
        {
            loop.boolValue = false;
        }
        if (record.boolValue && (current_recordValue != record.boolValue))
        {
            read.boolValue = false;
            current_readValue = false;
            loop.boolValue = false;
            current_recordValue = record.boolValue;
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
                        videoFile.stringValue = dragged_object;
                    }
                }
                break;
        }
    }
}
