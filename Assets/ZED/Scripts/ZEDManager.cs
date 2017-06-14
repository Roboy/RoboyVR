using UnityEngine;
using System;

public class ZEDManager : MonoBehaviour
{
    /// <summary>
    /// Current instance of the ZED Camera
    /// </summary>
    public sl.ZEDCamera zedCamera;

    [Header("Camera")]
    /// <summary>
    /// Selected resolution
    /// </summary>
    public sl.RESOLUTION videoMode = sl.RESOLUTION.HD720;
    /// <summary>
    /// Targeted FPS
    /// </summary>
    private float FPS = 0.0f;
    /// <summary>
    /// Depth mode
    /// </summary>
    public sl.DEPTH_MODE depthMode = sl.DEPTH_MODE.PERFORMANCE;
    /// <summary>
    /// Enable camera overlay
    /// </summary>
    [Tooltip("Enable ZED images")]
    public bool videoOverlay = true;
    /// <summary>
    /// Enable grab threading
    /// </summary>
    [Tooltip("Thread grab calls. May increase performance for multi-core processors.")]
    public bool multithreading = true;

    [Header("Motion Tracking")]
    /// <summary>
    /// Initialize the arrays
    /// </summary>
    public bool tracking = true;

    private bool isThreaded = false;

    private Quaternion orientation;
    private Vector3 position;

    private SVOManager ZedSVOManager;
    private Vector3 positionInit = new Vector3();
    private Quaternion rotationInit = Quaternion.identity;
    private bool spatialMemory = true;
    void Awake()
    {
        positionInit = transform.localPosition;
        rotationInit = transform.localRotation;
        position = positionInit;
        orientation = Quaternion.identity;

        isThreaded = multithreading;
        zedCamera = sl.ZEDCamera.GetInstance();

        ZedSVOManager = GetComponent<SVOManager>();
        if (ZedSVOManager != null)
        {
            //Create a camera
            if ((ZedSVOManager.read || ZedSVOManager.record) && ZedSVOManager.videoFile.Length == 0)
            {
                ZedSVOManager.record = false;
                ZedSVOManager.read = false;
            }
            if (ZedSVOManager.read)
            {
                ZedSVOManager.record = false;
                zedCamera.CreateCameraSVO(ZedSVOManager.videoFile);
            }
            else
            {
                zedCamera.CreateCameraLive(videoMode, FPS);
            }
        }
        else
        {
            zedCamera.CreateCameraLive(videoMode, FPS);
        }

        Debug.Log("ZED SDK Version " + sl.ZEDCamera.GetSDKVersion());

        //Initialize the camera in performance mode.
        sl.ERROR_CODE e = zedCamera.Init(depthMode);

        if (e != sl.ERROR_CODE.SUCCESS)
        {
            throw new Exception("Initialization failed " + e.ToString());
        }


        if (ZedSVOManager != null)
        {
            if (ZedSVOManager.record)
            {
                if (zedCamera.EnableRecording(ZedSVOManager.videoFile) != sl.ERROR_CODE.SUCCESS)
                {
                    ZedSVOManager.record = false;
                }
            }
        }
    }



    private void Start()
    {
        if (tracking && UnityEngine.VR.VRSettings.enabled && multithreading)
        {
            multithreading = false;
            isThreaded = false;
            Debug.Log("[ZED Plugin] : Multi-threading is deactivated with VR");
        }

        zedCamera.SetMultiThread(isThreaded);
        if (tracking)
        {
            //Enables the tracking and initializes the first position of the camera
            Quaternion quat = Quaternion.identity;
            Vector3 vec = new Vector3(0, 0, 0);
            if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, spatialMemory) == sl.ERROR_CODE.SUCCESS)))
            {
                throw new Exception("Error, tracking not available");
            }
        }

#if UNITY_EDITOR
        UnityEditor.EditorApplication.playmodeStateChanged = HandleOnPlayModeChanged;
#endif
    }

#if UNITY_EDITOR
    void HandleOnPlayModeChanged()
    {
        if (zedCamera == null) return;
        // This method is run whenever the play mode state is changed.
        if (UnityEditor.EditorApplication.isPaused)
        {
            zedCamera.SetPauseThread(true);
        }
        else
        {
            zedCamera.SetPauseThread(false);
        }
    }
#endif

    /// <summary>
    /// Get the tracking position from the ZED and update the manager's position
    /// </summary>
    private void UpdateTracking()
    {
        zedCamera.GetPosition(ref orientation, ref position);

        transform.localRotation = orientation;
        transform.localPosition = position;
    }

    // Update is called once per frame
    void Update()
    {
        if (zedCamera != null)
        {
            if (isThreaded || zedCamera.Grab() == 0)
            {
                if (tracking)
                {
                    UpdateTracking();
                }

                if (videoOverlay)
                {
                    zedCamera.UpdateTextures();
                }


                if (ZedSVOManager != null)
                {
                    if (ZedSVOManager.record)
                    {
                        zedCamera.Record();
                    }

                    if (ZedSVOManager.read && ZedSVOManager.loop)
                    {
                        if (zedCamera.GetSVOPosition() >= zedCamera.GetSVONumberOfFrames() - 2)
                        {
                            zedCamera.SetSVOPosition(0);

                            if (tracking)
                            {
                                zedCamera.DisableTracking();
                                Quaternion quat = Quaternion.identity;
                                Vector3 vec = new Vector3(0, 0, 0);
                                if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, spatialMemory) == sl.ERROR_CODE.SUCCESS)))
                                {
                                    throw new Exception("Error, tracking not available");
                                }
                                transform.localPosition = positionInit;
                                transform.localRotation = rotationInit;
                            }
                        }

                        
                    }
                }

            }
        }
    }


    void OnApplicationQuit()
    {
        if (zedCamera != null)
        {
            zedCamera.DisableTracking();
            if (ZedSVOManager != null)
            {
                if (ZedSVOManager.record)
                {
                    zedCamera.DisableRecording();
                }
            }
            zedCamera.Destroy();
        }
        zedCamera = null;
    }
}
