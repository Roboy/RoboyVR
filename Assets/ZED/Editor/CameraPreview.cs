using UnityEngine;
using UnityEditor;

class CameraPreview : EditorWindow
{

    public Camera camera;
    private RenderTexture renderTexture;
    static Material mat = null;

    private static int tabHeight = 10;
    private static Texture2D tex;

    private static bool isOpen { get; set; }
    public static bool IsOpen
    {
        get { return isOpen; }
    }

    static private Rect oldPos;
    static private Vector2 oldMinSize;
    static private Vector2 oldMaxSize;
    static GUIStyle textStyle = new GUIStyle();
    static public bool isFullScreen = false;
    static Color color;
    static public Vector2 customResolution = new Vector2(1280,720);
    static void Init()
    {
        var editorWindow = (EditorWindow)GetWindow<CameraPreview>(typeof(CameraPreview));
        textStyle.normal.textColor = Color.white;
        editorWindow.autoRepaintOnSceneChange = true;
        editorWindow.titleContent = new GUIContent("Camera Preview");
        color = Color.white;
        if (tex == null)
        {
            CreateBg();
        }
        oldPos = new Rect(0,0,1280,720);
        oldMinSize = new Vector2(672,376);
        oldMaxSize = editorWindow.maxSize;
    }

    static void CreateBg()
    {

        tex = new Texture2D(1, 1, TextureFormat.RGBA32, false);
        tex.SetPixel(0, 0, new Color(0.0f, 0.0f, 0.0f));
        tex.Apply();
    }
        


    public static void SetGameWindow(bool fullScreen = false)
    {
        isFullScreen = fullScreen;
        color = Color.white;
        
        tabHeight = 10;

        var editorWindow = (EditorWindow)GetWindow<CameraPreview>(typeof(CameraPreview));

        oldPos = editorWindow.position;
        oldMinSize = editorWindow.minSize;
        oldMaxSize = editorWindow.maxSize;

        Rect newPos = new Rect(0, 0 - tabHeight, Screen.currentResolution.width, Screen.currentResolution.height + tabHeight);
        newPos.x = Mathf.Sign(oldPos.x) * (int)Mathf.Round(Mathf.Abs(oldPos.x) / Screen.currentResolution.width) * Screen.currentResolution.width;

        if (isFullScreen)
        {
            editorWindow.position = newPos;
            editorWindow.minSize = new Vector2(Screen.currentResolution.width, Screen.currentResolution.height + tabHeight);
            editorWindow.maxSize = editorWindow.minSize;
            editorWindow.position = newPos;
        }
        else
        {
            editorWindow.minSize = new Vector2(customResolution.x, customResolution.y + tabHeight);
            editorWindow.maxSize = editorWindow.minSize;
            
        }

    }

    public static void Resize()
    {
        isFullScreen = false;
        var editorWindow = (EditorWindow)GetWindow<CameraPreview>(typeof(CameraPreview));
        editorWindow.position = oldPos;
        editorWindow.minSize = oldMinSize;
        editorWindow.maxSize = oldMaxSize;
        editorWindow.position = oldPos;
       
    }

    void Update()
    {
        
        isOpen = true;
        if (camera != null)
        {
            EnsureRenderTexture();
            camera.targetTexture = renderTexture;

            if (renderTexture != null)
            {
                camera.Render();
                camera.targetTexture = null;
                Repaint();
            }
        }
    }

    public void Create()
    {
        isOpen = true;
        if (camera != null)
        {
            renderTexture = new RenderTexture((int)(position.height * camera.aspect), (int)position.height, 32, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
        }
    }

    private void OnDestroy()
    {
        isOpen = false;
       if(renderTexture != null && renderTexture.IsCreated())
        {
            renderTexture.Release();
        }
    }


    private void OnFocus()
    {
        color = Color.white;
    }

    void EnsureRenderTexture()
    {
        if (renderTexture == null
            || (int)position.width != renderTexture.width
            || (int)position.height != renderTexture.height)
        {
            if (mat == null)
            {
                mat = new Material(Shader.Find("ZED/ZED_PreviewShader"));
            }

            if (renderTexture != null) renderTexture.Release();
           
            renderTexture = new RenderTexture((int)(position.height*camera.aspect), (int)position.height, 32, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
        }
    }



    void OnGUI()
    {
        Event e = Event.current;
        switch (e.type)
        {
            case EventType.KeyDown:
                {
                    if (Event.current.keyCode == (KeyCode.Escape))
                    {
                        Resize();
                    }
                    break;
                }
        }
        if (tex == null)
        {
            CreateBg();
        }
      
        GUI.DrawTexture(new Rect(0, 0, maxSize.x, maxSize.y), tex, ScaleMode.StretchToFill);

        if (renderTexture != null)
        {
            Graphics.DrawTexture(new Rect(0, position.height/2.0f - 0.5f*position.width / (16.0f / 9.0f) + tabHeight, position.width, position.width / (16.0f/9.0f)), renderTexture, mat);
        }
        if (isFullScreen)
        {
            color.a -= easeIn(0.1f, 0, 1, 1.5f);
            textStyle.normal.textColor = color;
            if (color.a < 0)
            {
                color.a = 0;
            }

            textStyle.fontSize = 30;
            GUI.Label(new Rect(position.width / 2.0f - 200 + 30, position.height / 2.0f - 15, 100, 30), "Press escape to exit", textStyle);
         }
        
    }

    float easeIn(float t, float b, float c, float d)
    {
        return -c * (Mathf.Sqrt(1 - (t /= d) * t) - 1) + b;
    }

}