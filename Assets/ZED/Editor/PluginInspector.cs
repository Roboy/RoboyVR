using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[InitializeOnLoad]
public class PluginInspector : EditorWindow
{
    static Texture2D image = null;

    private static EditorWindow window;
    private static string errorMessage = "";
    static void Init()
    {
        window = GetWindow<PluginInspector>(true);
        
        window.Show();
    }

    static PluginInspector()
    {
        EditorApplication.update += Update;
    }

    static void Update()
    {

        errorMessage = sl.ZEDCamera.CheckPlugin();

        if (errorMessage != "")
        {
            window = GetWindow<PluginInspector>(true);
            window.maxSize = new Vector2(400, 500);
            window.minSize = window.maxSize;
            window.Show(true);

        }
        EditorApplication.update -= Update;
    }

    void OnGUI()
    {
        if (image == null)
        {
            image = Resources.Load("Textures/logo", typeof(Texture2D)) as Texture2D;
           
        }
        var rect = GUILayoutUtility.GetRect(position.width, 150, GUI.skin.box);

        if (image)
        {
            GUI.DrawTexture(rect, image, ScaleMode.ScaleToFit);
        }
        GUIStyle myStyle = new GUIStyle(GUI.skin.label);
        myStyle.normal.textColor = Color.red;
        myStyle.fontStyle = FontStyle.Bold;

        GUILayout.Space(20);
        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        GUILayout.Label("Error ZED Unity Wrapper", myStyle);
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        myStyle = new GUIStyle(GUI.skin.box);
        myStyle.normal.textColor = Color.red;

        GUI.Box(new Rect(0, position.height/2, position.width, 100), errorMessage, myStyle);
         


        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Close"))
        {
            this.Close();
        }

    }
}

