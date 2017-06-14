using UnityEngine;
using UnityEditor;

public class ZEDEditor : EditorWindow
{
    private const int cbrightness = 4;
    private const int ccontrast = 4;
    private const int chue = 0;
    private const int csaturation = 4;
    private const int cwhiteBalance = 2600;

    static private int brightness = 4;
    static private int contrast = 4;
    static private int hue = 0;
    static private int saturation = 4;

    static private int gain;
    static private int exposure;
    static private int whiteBalance = cwhiteBalance;
    static private bool groupAuto = true;

    private const int refreshRate = 10;
    static private int refreshCount = 0;
    static bool setManualValue = true;

    static Color defaultColor;
    static GUIStyle style = new GUIStyle();

    static private sl.ZEDCamera zedCamera;

    static int tab = 0;


    static bool isInit = false;

    static private GUILayoutOption[] optionsButton = { GUILayout.MaxWidth(100) };

    static sl.CameraInformations parameters = new sl.CameraInformations();

    public ZEDEditor()
    {
        zedCamera = sl.ZEDCamera.GetInstance();
        EditorApplication.playmodeStateChanged += Draw;
    }

    void Draw()
    {
        if (zedCamera != null && Application.isPlaying)
            parameters = zedCamera.GetCameraInformation ();
        this.Repaint();
    }

    [MenuItem("Window/ZED Camera")]
    static void Init()
    {
        // Get existing open window or if none, make a new one:
        ZEDEditor window = (ZEDEditor)EditorWindow.GetWindow(typeof(ZEDEditor), false, "ZED Camera");

        style.normal.textColor = Color.red;
        style.fontSize = 15;
        style.margin.left = 5;

        parameters = new sl.CameraInformations();
        window.Show();

    }

    void GetAllSettingsCamera()
    {
        brightness = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.BRIGHTNESS);
        contrast = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.CONTRAST);
        hue = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.HUE);
        saturation = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.SATURATION);
        gain = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.GAIN);
        exposure = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE);
        whiteBalance = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE);
    }

    void OnFocus()
    {

        if (zedCamera != null && zedCamera.CameraIsReady)
        {
            parameters = zedCamera.GetCameraInformation();

            GetAllSettingsCamera();
        }
    }



    void FirstInit()
    {
        if (!isInit)
        {
            if (zedCamera != null && zedCamera.CameraIsReady)
            {
                isInit = true;

                GetAllSettingsCamera();

                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.GAIN, gain, true);
                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE, exposure, true);
                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE, -1, false);
                parameters = zedCamera.GetCameraInformation();
            }
        }
    }

    private void OnInspectorUpdate()
    {
        Repaint();
    }

    void CameraSettingsView()
    {
        GUILayout.Label("Video Mode", EditorStyles.boldLabel);

        if (zedCamera != null && zedCamera.CameraIsReady)
        {
            EditorGUILayout.LabelField("Resolution ", zedCamera.ImageWidth + " x " + zedCamera.ImageHeight);
            EditorGUILayout.LabelField("FPS ", zedCamera.GetCameraFPS().ToString());
        }
        else
        {
            EditorGUILayout.LabelField("Resolution ", 0 + " x " + 0);
            EditorGUILayout.LabelField("FPS ", "0");
        }
        EditorGUI.indentLevel = 0;
        GUILayout.Space(20);
        GUILayout.Label("Settings", EditorStyles.boldLabel);


        EditorGUI.BeginChangeCheck();
        brightness = EditorGUILayout.IntSlider("Brightness", brightness, 0, 8);
        if (EditorGUI.EndChangeCheck())
        {
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.BRIGHTNESS, brightness, false);
        }

        EditorGUI.BeginChangeCheck();
        contrast = EditorGUILayout.IntSlider("Contrast", contrast, 0, 8);
        if (EditorGUI.EndChangeCheck())
        {
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.CONTRAST, contrast, false);
        }

        EditorGUI.BeginChangeCheck();
        hue = EditorGUILayout.IntSlider("Hue", hue, 0, 11);
        if (EditorGUI.EndChangeCheck())
        {
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.HUE, hue, false);
        }

        EditorGUI.BeginChangeCheck();
        saturation = EditorGUILayout.IntSlider("Saturation", saturation, 0, 8);
        if (EditorGUI.EndChangeCheck())
        {
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.SATURATION, saturation, false);
        }
        EditorGUI.BeginChangeCheck();
        var origFontStyle = EditorStyles.label.fontStyle;
        EditorStyles.label.fontStyle = FontStyle.Bold;
        GUILayout.Space(20);
        groupAuto = EditorGUILayout.Toggle("Automatic", groupAuto, EditorStyles.toggle);
        if (!groupAuto && setManualValue)
        {
            //zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE, whiteBalance, false);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.GAIN, gain, false);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE, exposure, false);
            setManualValue = false;
        }
        if (groupAuto)
        {
            setManualValue = true;
        }
        EditorStyles.label.fontStyle = origFontStyle;

        GUI.enabled = !groupAuto;
        EditorGUI.BeginChangeCheck();
        EditorGUI.BeginChangeCheck();
        whiteBalance = 100 * EditorGUILayout.IntSlider("White balance", whiteBalance / 100, 26, 65);
        if (EditorGUI.EndChangeCheck())
        {
            if (!groupAuto)
            {
                Debug.Log(whiteBalance);
                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE, whiteBalance, false);
            }
        }
        gain = EditorGUILayout.IntSlider("Gain", gain, 0, 100);
        if (EditorGUI.EndChangeCheck())
        {
            if (!groupAuto)
            {
                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.GAIN, gain, false);
            }
        }
        EditorGUI.BeginChangeCheck();
        exposure = EditorGUILayout.IntSlider("Exposure", exposure, 0, 100);
        if (EditorGUI.EndChangeCheck())
        {
            if (!groupAuto)
            {
                zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE, exposure, false);
            }
        }

        refreshCount++;
        if (refreshCount >= refreshRate)
        {
            if (zedCamera != null && zedCamera.CameraIsReady)
            {
                exposure = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE);
                gain = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.GAIN);
                refreshCount = 0;
            }
        }

        GUI.enabled = true;
        EditorGUI.indentLevel = 0;

        GUILayout.FlexibleSpace();
        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Reset", optionsButton))
        {
            brightness = cbrightness;
            contrast = ccontrast;
            hue = chue;
            saturation = csaturation;

            groupAuto = true;
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.BRIGHTNESS, cbrightness, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.CONTRAST, ccontrast, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.HUE, hue, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.SATURATION, csaturation, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE, cwhiteBalance, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.GAIN, gain, true);
            zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE, exposure, true);

        }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        if (zedCamera != null && zedCamera.CameraIsReady)
        {
            if (EditorGUI.EndChangeCheck())
            {

                if (groupAuto)
                {
                    zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.GAIN, gain, true);
                    zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE, exposure, true);
                    zedCamera.SetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE, -1, true);

                    gain = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.GAIN);
                    exposure = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.EXPOSURE);
                    whiteBalance = zedCamera.GetCameraSettings(sl.CAMERA_SETTINGS.WHITEBALANCE);

                    Repaint();
                }
            }
        }
    }

    void LabelHorizontal(string name, float value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(name);
        GUILayout.Box(value.ToString());
        GUILayout.EndHorizontal();
    }

    void CalibrationSettingsView()
    {

        GUILayout.BeginHorizontal();

        GUILayout.BeginVertical();
        GUILayout.BeginVertical();
        GUILayout.Label("Left camera", EditorStyles.boldLabel);
        GUILayout.EndVertical();
        GUILayout.BeginVertical(EditorStyles.helpBox);

        LabelHorizontal("fx", parameters.calibParameters.leftCam.fx);
        LabelHorizontal("fy", parameters.calibParameters.leftCam.fy);
        LabelHorizontal("cx", parameters.calibParameters.leftCam.cx);
        LabelHorizontal("cy", parameters.calibParameters.leftCam.cy);

        GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
        LabelHorizontal("k1", (float)parameters.calibParameters.leftCam.disto[0]);
        LabelHorizontal("k2", (float)parameters.calibParameters.leftCam.disto[1]);
        GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
        LabelHorizontal("vFOV", parameters.calibParameters.leftCam.vFOV);
        LabelHorizontal("hFOV", parameters.calibParameters.leftCam.hFOV);

        GUILayout.FlexibleSpace();
        GUILayout.EndVertical();
        GUILayout.EndVertical();

        GUILayout.BeginVertical();
        GUILayout.BeginVertical();
        GUILayout.Label("Right camera", EditorStyles.boldLabel);
        GUILayout.EndVertical();
        GUILayout.BeginVertical(EditorStyles.helpBox);

        LabelHorizontal("fx", parameters.calibParameters.rightCam.fx);
        LabelHorizontal("fy", parameters.calibParameters.rightCam.fy);
        LabelHorizontal("cx", parameters.calibParameters.rightCam.cx);
        LabelHorizontal("cy", parameters.calibParameters.rightCam.cy);

        GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
        LabelHorizontal("k1", (float)parameters.calibParameters.rightCam.disto[0]);
        LabelHorizontal("k2", (float)parameters.calibParameters.rightCam.disto[1]);
        GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));
        LabelHorizontal("vFOV", parameters.calibParameters.rightCam.vFOV);
        LabelHorizontal("hFOV", parameters.calibParameters.rightCam.hFOV);

        GUILayout.FlexibleSpace();
        GUILayout.EndVertical();
        GUILayout.EndVertical();

        GUILayout.BeginVertical();

        GUILayout.Label("Stereo", EditorStyles.boldLabel);

        GUILayout.BeginVertical(EditorStyles.helpBox);
        LabelHorizontal("Baseline", parameters.calibParameters.Trans[0]);
        LabelHorizontal("Convergence", parameters.calibParameters.Rot[1]);
        GUILayout.EndVertical();

        GUILayout.Label("Optional", EditorStyles.boldLabel);
        GUILayout.BeginVertical(EditorStyles.helpBox);
        LabelHorizontal("Rx", parameters.calibParameters.Rot[0]);
        LabelHorizontal("Rz", parameters.calibParameters.Rot[2]);
        GUILayout.EndVertical();

        GUILayout.EndVertical();

        GUILayout.EndHorizontal();
    }

    void OnGUI()
    {
        FirstInit();
        defaultColor = GUI.color;
        if (zedCamera != null && zedCamera.CameraIsReady)
            GUI.color = Color.green;
        else GUI.color = Color.red;
        GUILayout.BeginHorizontal(EditorStyles.helpBox);
        GUILayout.FlexibleSpace();
        if (zedCamera != null && zedCamera.CameraIsReady)
        {
            style.normal.textColor = Color.black;
            GUILayout.Label("Connected", style);
        }
        else
        {
            style.normal.textColor = Color.black;
            GUILayout.Label("Not Connected", style);
        }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        GUI.color = defaultColor;
        EditorGUI.BeginChangeCheck();
        tab = GUILayout.Toolbar(tab, new string[] { "Camera Control", "Calibration" });
        if (EditorGUI.EndChangeCheck())
        {
            if (zedCamera != null && zedCamera.CameraIsReady)
            {
                parameters = zedCamera.GetCameraInformation();
            }
        }
        switch (tab)
        {
            case 0:
                CameraSettingsView();
                break;

            case 1:
                CalibrationSettingsView();
                break;

            default:
                CameraSettingsView();
                break;
        }
    }
}
