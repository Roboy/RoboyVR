using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
public class TextureOverlay : MonoBehaviour
{
    /// <summary>
    /// The screen is the quad where the textures are displayed
    /// </summary>
    public GameObject canvas;
    /// <summary>
    /// It's the main material, used to set the color and the depth
    /// </summary>
    private Material matRGB;

    /// <summary>
    /// All the textures displayed are in 16/9
    /// </summary>
    private float aspect = 16.0f/9.0f;

    /// <summary>
    /// The main camera is the camera controlled by the ZED
    /// </summary>
    private Camera mainCamera;

    //[Tooltip("Set the video type at the initialization")]
    private sl.VIEW videoType;

    [HideInInspector]
    public Texture2D camZedLeft;

    Texture2D depthXYZZed;
    CommandBuffer buffer;

    //public RenderTexture mask;
    void Awake()
    {
        mainCamera = GetComponent<Camera>();
        Hide();
        mainCamera.aspect = aspect;
    }

    /// <summary>
    /// Hide the screen to any other cameras
    /// </summary>
    private void Hide()
    {
        gameObject.transform.GetChild(0).gameObject.layer = 20;
        foreach (Camera c in Camera.allCameras)
        {
            if (c != mainCamera)
            {
                c.cullingMask = ~(1 << 20);
            }
        }
    }

    public void setVideoType(sl.VIEW view)
    {
        videoType = view;
    }

    void Start()
    {
        //Set textures to the shader
        matRGB = canvas.GetComponent<Renderer>().material;
        sl.ZEDCamera zedCamera = sl.ZEDCamera.GetInstance();
        if (videoType == sl.VIEW.LEFT_GREY || videoType == sl.VIEW.RIGHT_GREY || videoType == sl.VIEW.LEFT_UNRECTIFIED_GREY || videoType == sl.VIEW.RIGHT_UNRECTIFIED_GREY)
        {
            matRGB.SetInt("_isGrey", 1);
        }
        else
        {
            matRGB.SetInt("_isGrey", 0);
        }

        //Create two textures and fill them with the ZED computed images
        camZedLeft = zedCamera.CreateTextureImageType(videoType);
        depthXYZZed = zedCamera.CreateTextureMeasureType(sl.MEASURE.XYZ);
        matRGB.SetTexture("_CameraTex", camZedLeft);
        matRGB.SetTexture("_DepthXYZTex", depthXYZZed);

        if (zedCamera.CameraIsReady)
        {
            mainCamera.fieldOfView = zedCamera.GetFOV() * Mathf.Rad2Deg;
            mainCamera.projectionMatrix = zedCamera.Projection;

            scale(canvas.gameObject, GetFOVFromProjectionMatrix(mainCamera.projectionMatrix));
        }
        else
        {
            scale(canvas.gameObject, mainCamera.fieldOfView);
        }
    }



    /// <summary>
    /// Get back the FOV from the Projection matrix, to bypass a round number
    /// </summary>
    /// <param name="projection"></param>
    /// <returns></returns>
    float GetFOVFromProjectionMatrix(Matrix4x4 projection)
    {
        return Mathf.Atan(1 / projection[1, 1]) * 2.0f;
    }


    /// <summary>
    /// Scale a screen in front of the camera, where all the textures will be rendered.
    /// </summary>
    /// <param name="screen"></param>
    /// <param name="fov"></param>
    private void scale(GameObject screen, float fov)
    {
        float height = Mathf.Tan(0.5f * fov) * 2.0f;
        screen.transform.localScale = new Vector3(height * aspect, height, 1);
    }
}

