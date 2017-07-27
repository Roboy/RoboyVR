using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(TextureOverlay))]
public class ScreenManagerEditor : Editor
{

    private TextureOverlay screen;
    public GameObject o;
    private const string OPEN_MESSAGE = "Open External Preview";
    private const string CLOSE_MESSAGE = "Close External Preview";

    enum PREVIEW_MODE
    {
        FULLSCREEN,
        WINDOWED
    };
    private PREVIEW_MODE previewMode;
    public void OnEnable()
    {
        screen = (TextureOverlay)target;
        o = screen.gameObject;
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (!CameraPreview.IsOpen)
        {
            if (GUILayout.Button(OPEN_MESSAGE))
            {
                CameraPreview window = (CameraPreview)EditorWindow.GetWindow(typeof(CameraPreview));
                window.Create();
                window.camera = o.GetComponent<Camera>();
                window.Show();
            }
        }
        else
        {
            if (GUILayout.Button(CLOSE_MESSAGE))
            {
                CameraPreview window = (CameraPreview)EditorWindow.GetWindow(typeof(CameraPreview));

                window.Close();
            }

            GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
            previewMode = (PREVIEW_MODE)EditorGUILayout.EnumPopup("Preview mode", previewMode);

             if(previewMode == PREVIEW_MODE.WINDOWED)
            {
                CameraPreview.customResolution = EditorGUILayout.Vector2Field("Custom resolution", CameraPreview.customResolution);
            }
            EditorGUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Apply"))
            {
               
                CameraPreview.SetGameWindow(previewMode == PREVIEW_MODE.FULLSCREEN);
            }
            EditorGUILayout.EndHorizontal();
        }

        /***** Uncomment to watch the lights taken in forward mode ****/
        /*
        for(int i = 0; i < screen.numberPointLights; i++)
        {
            TextureOverlay.PointLight p = screen.pointLights[i];
            EditorGUILayout.LabelField("Point light : pos " + p.position + "|| range " + p.range);
        }

        for (int i = 0; i < screen.numberSpotLights; i++)
        {
            TextureOverlay.SpotLight p = screen.spotLights[i];
            EditorGUILayout.LabelField("Spot light : pos " + p.position + " || direction" + p.direction);
        }
        */
    }
}
