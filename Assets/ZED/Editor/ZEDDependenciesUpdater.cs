
using UnityEngine;
using UnityEditor;

[InitializeOnLoad]
public class ZEDDependenciesUpdater : EditorWindow
{
    const string pluginUrl = "http://u3d.as/content/valve-corporation/steam-vr-plugin";
    const string packageName = "SteamVR";
    const string defineName = "ZED_STEAM_VR";

    static ZEDDependenciesUpdater window;
    static Texture2D image = null;
    bool toggleState;

    static ZEDDependenciesUpdater()
    {
       
        EditorApplication.update += Update;
    }

    public static bool CheckPackageExists(string name)
    {
        string[] packages = AssetDatabase.FindAssets(name);
        return packages.Length != 0 && AssetDatabase.IsValidFolder("Assets/" + name);
    }

    public static void ActivateDefine()
    {
        string defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(BuildTargetGroup.Standalone);
        if (defines.Length != 0)
        {
            if (!defines.Contains(defineName))
            {
                defines += ";" + defineName;
            }
        }
        else
        {
            if (!defines.Contains(defineName))
            {
                defines += defineName;
            }
        }
        PlayerSettings.SetScriptingDefineSymbolsForGroup(BuildTargetGroup.Standalone, defines);
    }

    public static void DesactivateDefine()
    {
        string defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(BuildTargetGroup.Standalone);
        if (defines.Length != 0)
        {
            if (defines.Contains(defineName))
            {
                defines = defines.Remove(defines.IndexOf(defineName), defineName.Length);

                if (defines.LastIndexOf(";") == defines.Length - 1 && defines.Length !=0)
                {
                    defines.Remove(defines.LastIndexOf(";"), 1);
                }
            }
        }
        PlayerSettings.SetScriptingDefineSymbolsForGroup(BuildTargetGroup.Standalone, defines);
    }

    static void Update()
    {
        if (!EditorPrefs.GetBool(packageName))
        {
            if (!CheckPackageExists(packageName))
            {
                window = GetWindow<ZEDDependenciesUpdater>(true);
            }
            else
            {
                ActivateDefine();
            }
        }
        EditorApplication.update -= Update;
    }

    private void OnGUI()
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
        EditorGUILayout.HelpBox("The GreenScreen scene needs the package SteamVR to work", MessageType.Warning);

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Get Latest Version"))
        {
            Application.OpenURL(pluginUrl);
            if (window != null)
            {
                window.Close();
            }
        }
        if (GUILayout.Button("Close"))
        {
            if (window != null)
            {
                window.Close();
            }
        }
        EditorGUILayout.EndHorizontal();
        EditorGUI.BeginChangeCheck();

        var doNotShow = GUILayout.Toggle(toggleState, "Do not prompt for this version again.");
        if (EditorGUI.EndChangeCheck())
        {
            toggleState = doNotShow;
            
            if (doNotShow)
                EditorPrefs.SetBool(packageName, true);
            else
                EditorPrefs.DeleteKey(packageName);
        }
    }
}
