using UnityEngine;
using UnityEngine.Rendering;
/// <summary>
/// Creates the textures used by the ZED. Set the depth after the Unity's depth (forward and deferred)
/// Manges the lights collected and enables the shadows
/// </summary>
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
    private float aspect = 16.0f / 9.0f;

    /// <summary>
    /// The main camera is the camera controlled by the ZED
    /// </summary>
    private Camera mainCamera;

    /// <summary>
    /// Main view
    /// </summary>
    private sl.VIEW videoType;

    /// <summary>
    /// Image Left of the ZED
    /// </summary>
    [HideInInspector]
    public Texture2D camZedLeft;

    /// <summary>
    /// Depth from the ZED or a texture XYZ
    /// </summary>
    private Texture2D depthXYZZed;

    /// <summary>
    /// Normals computed from the ZED
    /// </summary>
    private Texture2D normals;

    /// <summary>
    /// Main CommandBuffer used to insert depth after Unity's buffer
    /// </summary>
    private CommandBuffer buffer;


    private Material forwardMat;
    private Material deferredMat;

    /******Point light**************/
    [SerializeField]
    public struct PointLight
    {
        public Vector4 color;
        public float range;
        public Vector3 position;
    }
    private const int NUMBER_POINT_LIGHT_MAX = 8;
    [SerializeField]
    public PointLight[] pointLights = new PointLight[NUMBER_POINT_LIGHT_MAX];
    private const int SIZE_POINT_LIGHT_BYTES = 32;
    private ComputeBuffer computeBuffePointLight;

    /**********Spot light************/
    [SerializeField]
    public struct SpotLight
    {
        public Vector4 color;
        public Vector3 position;
        public Vector4 direction;
        public Vector4 parameters;
    }
    private const int NUMBER_SPOT_LIGHT_MAX = 8;
    [SerializeField]
    public SpotLight[] spotLights = new SpotLight[NUMBER_SPOT_LIGHT_MAX];
    private const int SIZE_SPOT_LIGHT_BYTES = 60;
    private const int NUMBER_LIGHTS_MAX = NUMBER_POINT_LIGHT_MAX / 2 + NUMBER_SPOT_LIGHT_MAX / 2;
    ComputeBuffer computeBufferSpotLight;

    //Activates the Depth retrieve with XYZ, it's slower but if a point cloud is available, less textures are used
    private bool xyz = false;

    /// <summary>
    /// Mesh of the frame, may be used to force shadows
    /// </summary>
    private MeshFilter mesh;

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
        gameObject.transform.GetChild(0).gameObject.layer = sl.ZEDCamera.Tag;
        foreach (Camera c in Camera.allCameras)
        {
            if (c != mainCamera)
            {
                c.cullingMask &= ~(1 << sl.ZEDCamera.Tag);
            }
        }
        mainCamera.cullingMask = ~(1 << sl.ZEDCamera.TagOneObject);
    }

    public void setVideoType(sl.VIEW view)
    {
        videoType = view;
    }

    public void OnEnable()
    {
        mesh = gameObject.transform.GetChild(0).GetComponent<MeshFilter>();
    }


    //Bounds around the camera, it filters which camera are taken in forward mode 
    private Bounds bounds;

    void Start()
    {

#if UNITY_EDITOR
        UnityEditor.PlayerSettings.SetAspectRatio(UnityEditor.AspectRatio.Aspect16by9, true);
        UnityEditor.PlayerSettings.SetAspectRatio(UnityEditor.AspectRatio.Aspect16by10, false);
        UnityEditor.PlayerSettings.SetAspectRatio(UnityEditor.AspectRatio.Aspect4by3, false);
        UnityEditor.PlayerSettings.SetAspectRatio(UnityEditor.AspectRatio.Aspect5by4, false);
#endif

        bounds = new Bounds(transform.position, new Vector3(30, 30, 30));

        sl.ZEDCamera zedCamera = sl.ZEDCamera.GetInstance();
        //Set the camera parameters and scale the screen
        if (zedCamera.CameraIsReady)
        {
            mainCamera.fieldOfView = zedCamera.GetFOV() * Mathf.Rad2Deg;
            mainCamera.projectionMatrix = zedCamera.Projection;
            mainCamera.nearClipPlane = 0.2f;
            mainCamera.farClipPlane = 500.0f;
            scale(canvas.gameObject, GetFOVFromProjectionMatrix(mainCamera.projectionMatrix));
        }
        else
        {
            scale(canvas.gameObject, mainCamera.fieldOfView);
        }

        if (xyz)
        {
            Shader.EnableKeyword("ZED_XYZ");
            depthXYZZed = zedCamera.CreateTextureMeasureType(sl.MEASURE.XYZ);
        }
        else
        {
            Shader.DisableKeyword("ZED_XYZ");
            depthXYZZed = zedCamera.CreateTextureMeasureType(sl.MEASURE.DEPTH);

        }
        camZedLeft = zedCamera.CreateTextureImageType(videoType);

        normals = zedCamera.CreateTextureMeasureType(sl.MEASURE.NORMALS);

        //If forward mode is activated
        if (mainCamera.actualRenderingPath == RenderingPath.Forward)
        {
            Shader.SetGlobalInt("_HasShadows", 0);

            gameObject.transform.GetChild(0).GetComponent<MeshRenderer>().enabled = true;

            //Set textures to the shader
            matRGB = canvas.GetComponent<Renderer>().material;
            matRGB.SetInt("_isLinear", System.Convert.ToInt32(QualitySettings.activeColorSpace));

            forwardMat = Resources.Load("Materials/Mat_ZED_Forward") as Material;

            if (videoType == sl.VIEW.LEFT_GREY || videoType == sl.VIEW.RIGHT_GREY || videoType == sl.VIEW.LEFT_UNRECTIFIED_GREY || videoType == sl.VIEW.RIGHT_UNRECTIFIED_GREY)
            {
                matRGB.SetInt("_isGrey", 1);
            }
            else
            {
                matRGB.SetInt("_isGrey", 0);
            }

            GameObject o = GameObject.CreatePrimitive(PrimitiveType.Quad);
            o.name = "ZED_FORCE_SHADOW";
            o.transform.parent = transform;
            o.transform.localScale = new Vector3(0.001f, 0.001f, 0.001f);
            o.transform.localPosition = new Vector3(0, 0, mainCamera.nearClipPlane);
            o.GetComponent<MeshRenderer>().sharedMaterial = Resources.Load("Materials/Mat_ZED_Nothing") as Material;
            Destroy(o.GetComponent<MeshCollider>());
            o.hideFlags = HideFlags.HideAndDontSave;

            matRGB.SetTexture("_MainTex", camZedLeft);
            matRGB.SetTexture("_CameraTex", camZedLeft);
            matRGB.SetTexture("_DepthXYZTex", depthXYZZed);
            matRGB.SetTexture("_NormalsTex", normals);

            forwardMat.SetTexture("_MainTex", camZedLeft);
            forwardMat.SetTexture("_DepthXYZTex", depthXYZZed);

            buffer = new CommandBuffer();
            buffer.name = "ZED_DEPTH";
            buffer.SetRenderTarget(BuiltinRenderTextureType.CurrentActive, BuiltinRenderTextureType.Depth);
            buffer.DrawMesh(mesh.mesh, gameObject.transform.GetChild(0).transform.localToWorldMatrix, forwardMat);

            // mainCamera.AddCommandBuffer(CameraEvent.BeforeDepthTexture, buffer);

            computeBuffePointLight = new ComputeBuffer(NUMBER_POINT_LIGHT_MAX, SIZE_POINT_LIGHT_BYTES);
            computeBuffePointLight.SetData(pointLights);
            matRGB.SetBuffer("pointLights", computeBuffePointLight);

            computeBufferSpotLight = new ComputeBuffer(NUMBER_SPOT_LIGHT_MAX, SIZE_SPOT_LIGHT_BYTES);
            computeBufferSpotLight.SetData(spotLights);
            matRGB.SetBuffer("spotLights", computeBufferSpotLight);

            zedCamera.SetDepthMaxRangeValue(15);
        }
        else if (mainCamera.actualRenderingPath == RenderingPath.DeferredShading)
        {
            //Sets the custom shader for the deferred pipeline
            GraphicsSettings.SetCustomShader(BuiltinShaderType.DeferredShading, (Resources.Load("Materials/Mat_ZED_Custom_Deferred") as Material).shader);

            deferredMat = Resources.Load("Materials/Mat_ZED_Deferred") as Material;

            deferredMat.SetTexture("_MainTex", camZedLeft);
            deferredMat.SetTexture("_DepthXYZTex", depthXYZZed);
            deferredMat.SetTexture("_NormalsTex", normals);

            //gameObject.transform.GetChild(0).GetComponent<MeshRenderer>().enabled = false;
            buffer = new CommandBuffer();
            buffer.name = "ZED_DEPTH";

            RenderTargetIdentifier[] mrt = { BuiltinRenderTextureType.GBuffer0, BuiltinRenderTextureType.GBuffer1, BuiltinRenderTextureType.GBuffer2, BuiltinRenderTextureType.GBuffer3 };
            buffer.SetRenderTarget(mrt, BuiltinRenderTextureType.CameraTarget);
            buffer.DrawMesh(mesh.mesh, gameObject.transform.GetChild(0).transform.localToWorldMatrix, deferredMat);

            //Set an object in the scene to force shadow casting
            gameObject.transform.GetChild(0).position = new Vector3(0, 0, mainCamera.nearClipPlane);
            gameObject.transform.GetChild(0).localScale = new Vector3(0.01f, 0.01f, 0.01f);
            gameObject.transform.GetChild(0).GetComponent<MeshRenderer>().sharedMaterial = Resources.Load("Materials/Mat_ZED_Nothing") as Material;

            buffer.SetRenderTarget(BuiltinRenderTextureType.CameraTarget);
            mainCamera.AddCommandBuffer(CameraEvent.AfterGBuffer, buffer);

        }
        else
        {
            Debug.LogError(" [ ZED Plugin ] : The rendering path " + mainCamera.actualRenderingPath.ToString() + " is not compatible with the ZED");
        }
    }


    [HideInInspector]
    public int numberPointLights;
    [HideInInspector]
    public int numberSpotLights;
    bool ghasShadows = false;

    void UpdateLights()
    {
        bool hasShadows = false;

        int pointLightIndex = 0;
        int spotLighIndex = 0;
        foreach (ZEDLight zed_light in ZEDLight.s_lights)
        {
            Light light = zed_light.cachedLight;

            if (light.type == LightType.Directional || bounds.SqrDistance(light.transform.position) < light.range)
            {

                //Desactivate all shadows from point light and spot light, they are not currently supported
                if (light.type != LightType.Directional)
                {
                    light.shadows = LightShadows.None;
                }
                if (zed_light.IsEnabled() && ((pointLightIndex + spotLighIndex) < NUMBER_LIGHTS_MAX || light.type == LightType.Directional))
                {
                    if (light.type == LightType.Point)
                    {

                        if (pointLightIndex < NUMBER_POINT_LIGHT_MAX)
                        {
                            pointLights[pointLightIndex].color = light.color * light.intensity;
                            pointLights[pointLightIndex].position = light.gameObject.transform.position;
                            pointLights[pointLightIndex].range = light.range;

                            pointLightIndex++;
                        }
                        
                    }

                    else if (light.type == LightType.Spot)
                    {

                        if (spotLighIndex < NUMBER_SPOT_LIGHT_MAX)
                        {
                            spotLights[spotLighIndex].color = light.color * light.intensity;
                            spotLights[spotLighIndex].position = light.gameObject.transform.position;
                            spotLights[spotLighIndex].direction = new Vector4(light.gameObject.transform.forward.normalized.x, light.gameObject.transform.forward.normalized.y, light.gameObject.transform.forward.normalized.z, Mathf.Cos((light.spotAngle / 2.0f) * Mathf.Deg2Rad));
                            spotLights[spotLighIndex].parameters = new Vector4(light.spotAngle, light.intensity, 1.0f / light.range, zed_light.interiorCone);
                            spotLighIndex++;
                        }
                    }
                    else if (light.type == LightType.Directional)
                    {
                        hasShadows = light.shadows != LightShadows.None && QualitySettings.shadows != ShadowQuality.Disable;

                        // Copy the shadows from the directional light, If not no shadows in transparent mode
                        if (light.commandBufferCount == 0)
                        {
                            CommandBuffer lightBuffer = new CommandBuffer();
                            lightBuffer.name = "ZED_Copy_ShadowMap";
                            lightBuffer.SetGlobalTexture("_DirectionalShadowMap", new RenderTargetIdentifier(BuiltinRenderTextureType.CurrentActive));

                            light.AddCommandBuffer(LightEvent.AfterScreenspaceMask, lightBuffer);
                        }
                    }
                }
            }
        }
        if (computeBuffePointLight != null)
        {
            computeBuffePointLight.SetData(pointLights);
        }
        if (computeBufferSpotLight != null)
        {
            computeBufferSpotLight.SetData(spotLights);
        }

        numberPointLights = pointLightIndex;
        numberSpotLights = spotLighIndex;
        if (matRGB != null)
        {
            if (hasShadows != ghasShadows)
            {
                ghasShadows = hasShadows;
                Shader.SetGlobalInt("_HasShadows", System.Convert.ToInt32(ghasShadows));
                mainCamera.RemoveCommandBuffer(CameraEvent.BeforeDepthTexture, buffer);
                if (hasShadows)
                {
                    mainCamera.AddCommandBuffer(CameraEvent.BeforeDepthTexture, buffer);
                }

            }
            matRGB.SetInt("numberPointLights", pointLightIndex);
            matRGB.SetInt("numberSpotLights", spotLighIndex);
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
        float height = Mathf.Tan(0.5f * fov) * 2.0f * Vector3.Distance(screen.transform.localPosition, mainCamera.transform.localPosition);
        screen.transform.localScale = new Vector3(height * aspect, height, 1);
    }

    void OnApplicationQuit()
    {

        if (mainCamera.actualRenderingPath == RenderingPath.Forward)
        {
            if (computeBuffePointLight != null)
            {
                computeBuffePointLight.Release();
            }

            if (computeBufferSpotLight != null)
            {
                computeBufferSpotLight.Release();
            }
        }
    }

    void Update()
    {

        if (mainCamera.actualRenderingPath == RenderingPath.Forward)
        {
            matRGB.SetVector("_CameraRotationQuat", new Vector4(mainCamera.transform.rotation.x, mainCamera.transform.rotation.y, mainCamera.transform.rotation.z, mainCamera.transform.rotation.w));
            //Bounds are not used
            bounds.center = transform.position;
            UpdateLights();
        }
        else if (mainCamera.actualRenderingPath == RenderingPath.DeferredShading)
        {
            deferredMat.SetVector("_CameraRotationQuat", new Vector4(mainCamera.transform.rotation.x, mainCamera.transform.rotation.y, mainCamera.transform.rotation.z, mainCamera.transform.rotation.w));
        }
    }

}
