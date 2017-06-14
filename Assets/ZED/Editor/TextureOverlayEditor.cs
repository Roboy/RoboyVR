using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(TextureOverlay))]
public class ScreenManagerEditor : Editor {

    private TextureOverlay screen;
    public GameObject o;
    private const string OPEN_MESSAGE = "Open External Preview";
    private const string CLOSE_MESSAGE = "Close External Preview";

    private bool boost;
    public void OnEnable()
    {
        screen = (TextureOverlay)target;
        o = screen.gameObject;
        boost =  PlayerSettings.colorSpace == ColorSpace.Linear;
        CameraPreview.Boost(boost);
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (!CameraPreview.IsOpen)
        {
            if (GUILayout.Button(OPEN_MESSAGE))
            {
                CameraPreview window = (CameraPreview)EditorWindow.GetWindow(typeof(CameraPreview));
                boost = PlayerSettings.colorSpace == ColorSpace.Linear;
                window.Create();
                window.camera = o.GetComponent<Camera>();
                window.Show();
                CameraPreview.Boost(boost);

            }
        }
        else
        {
            if (GUILayout.Button(CLOSE_MESSAGE))
            {
                CameraPreview window = (CameraPreview)EditorWindow.GetWindow(typeof(CameraPreview));

                window.Close();
            }
            EditorGUI.BeginChangeCheck();
            GUIContent content = new GUIContent("Gamma Correction", "Correct the gamma color applied by the HMD");
            boost = EditorGUILayout.Toggle(content, boost);
            CameraPreview.Boost(boost);

            if (EditorGUI.EndChangeCheck())
            {
                CameraPreview.Boost(boost);
            }

            EditorGUI.BeginChangeCheck();
            content = new GUIContent("Full Screen", "Press escape to exit full screen mode");
            CameraPreview.isFullScreen = EditorGUILayout.Toggle(content, CameraPreview.isFullScreen);
            if (EditorGUI.EndChangeCheck())
            {
                if (CameraPreview.isFullScreen)
                {
                    CameraPreview.FullScreenGameWindow();
                }
                else
                {
                    CameraPreview.CloseGameWindow();
                }
            }
        }
    }
}
