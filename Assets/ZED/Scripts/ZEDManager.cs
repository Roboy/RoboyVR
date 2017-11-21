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
    public sl.RESOLUTION resolution = sl.RESOLUTION.HD720;
    /// <summary>
    /// Targeted FPS
    /// </summary>
    private float FPS = 0.0f;
    /// <summary>
    /// Depth mode
    /// </summary>
    public sl.DEPTH_MODE depthMode = sl.DEPTH_MODE.PERFORMANCE;
    /// <summary>
    /// Sensing mode
    /// </summary>
    public sl.SENSING_MODE sensingMode = sl.SENSING_MODE.FILL;
    /// <summary>
    /// Enable camera overlay
    /// </summary>
    [Tooltip("Enable ZED images")]
    public bool videoOverlay = true;
    [HideInInspector]
    public bool depthStabilizer = true;
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

    private bool isMoving = false;
    private bool isThreaded = false;
    private bool isTrackingEnable = false;

    private Quaternion orientation;
    private Vector3 position;

    private SVOManager zedSVOManager;
    private Vector3 positionInit = new Vector3();
    private Quaternion rotationInit = Quaternion.identity;


    private bool spatialMemory = true;
    private Transform cameraLeft;

#if UNITY_EDITOR
    void OnValidate()
    {
        if (zedCamera != null)
        {

            if (!isTrackingEnable && tracking)
            {
                //Enables the tracking and initializes the first position of the camera
                Quaternion quat = Quaternion.identity;
                Vector3 vec = new Vector3(0, 0, 0);
                if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, spatialMemory) == sl.ERROR_CODE.SUCCESS)))
                {
                    depthStabilizer = false;
                    throw new Exception("Error, tracking not available");
                }
                else
                {
                    isMoving = true;
                    isTrackingEnable = true;
                }
            }

            if (isThreaded != multithreading)
            {
                if (tracking && UnityEngine.XR.XRSettings.enabled && multithreading)
                {
                    Debug.Log("[ZED Plugin] : Multi-threading is deactivated with VR");
                    multithreading = false;
                    isThreaded = false;
                }
                else
                {
                    ZEDUpdater.GetInstance().SetMultiThread(multithreading);
                    isThreaded = multithreading;
                }
            }
        }

    }
#endif


    void Awake()
    {
        isMoving = tracking;

        positionInit = transform.localPosition;
        rotationInit = transform.localRotation;
        position = positionInit;
        orientation = Quaternion.identity;

        isThreaded = multithreading;
        zedCamera = sl.ZEDCamera.GetInstance();
        zedSVOManager = GetComponent<SVOManager>();
        if (zedSVOManager != null)
        {
            //Create a camera
            if ((zedSVOManager.read || zedSVOManager.record) && zedSVOManager.videoFile.Length == 0)
            {
                zedSVOManager.record = false;
                zedSVOManager.read = false;
            }
            if (zedSVOManager.read)
            {
                zedSVOManager.record = false;
                zedCamera.CreateCameraSVO(zedSVOManager.videoFile);
            }
            else
            {
                zedCamera.CreateCameraLive(resolution, FPS);
            }
        }
        else
        {

            zedCamera.CreateCameraLive(resolution, FPS);
        }

        Debug.Log("ZED SDK Version " + sl.ZEDCamera.GetSDKVersion());

        //Initialize the camera in performance mode.
        sl.ERROR_CODE e = zedCamera.Init(depthMode);

        if (e != sl.ERROR_CODE.SUCCESS)
        {
            throw new Exception("Initialization failed " + e.ToString());
        }


        if (zedSVOManager != null)
        {
            if (zedSVOManager.record)
            {

                if (zedCamera.EnableRecording(zedSVOManager.videoFile) != sl.ERROR_CODE.SUCCESS)
                {
                    zedSVOManager.record = false;
                }
            }


            if (zedSVOManager.read)
            {
                zedSVOManager.NumberFrameMax = zedCamera.GetSVONumberOfFrames();
            }

        }

        Component[] cams = gameObject.transform.GetComponentsInChildren(typeof(Camera));
        foreach (Camera cam in cams)
        {
            if (cam.stereoTargetEye == StereoTargetEyeMask.None)
            {
                cameraLeft = cam.transform;
               // cam.cullingMask &= ~(1 << sl.ZEDCamera.TagOneObject);
            }
        }

    }

    public Transform GetLeftCameraTransform()
    {
        return cameraLeft;
    }


    private void Start()
    {

        if (tracking && UnityEngine.XR.XRSettings.enabled && multithreading)
        {
            multithreading = false;
            isThreaded = false;
            Debug.Log("[ZED Plugin] : Multi-threading is deactivated with VR");
        }

        ZEDUpdater.GetInstance().SetMultiThread(isThreaded);
        if (!isTrackingEnable && tracking)
        {
            //Enables the tracking and initializes the first position of the camera
            Quaternion quat = Quaternion.identity;
            Vector3 vec = new Vector3(0, 0, 0);
            if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, spatialMemory) == sl.ERROR_CODE.SUCCESS)))
            {
                depthStabilizer = false;
                throw new Exception("Error, tracking not available");
            }
            else
            {
                isMoving = true;
                isTrackingEnable = true;
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
            ZEDUpdater.GetInstance().SetPauseThread(true);
        }
        else if (zedSVOManager != null && !zedSVOManager.pause)
        {
            ZEDUpdater.GetInstance().SetPauseThread(false);
        }

#if UNITY_EDITOR
        UnityEditor.EditorApplication.playmodeStateChanged = HandleOnPlayModeChanged;
#endif
    }



#endif
    /// <summary>
    /// Get the tracking position from the ZED and update the manager's position
    /// </summary>
    private void UpdateTracking()
    {
        zedCamera.GetPosition(ref orientation, ref position);
        if (isMoving)
        {
            transform.localRotation = orientation;
            transform.localPosition = position;
        }
    }




    // Update is called once per frame
    void Update()
    {
        ZEDUpdater.GetInstance().Update(sensingMode);
    }


    public void OnEnable()
    {
        ZEDUpdater.OnGrab += Grab;
    }

    public void OnDisable()
    {
        ZEDUpdater.OnGrab -= Grab;
    }



    public void Grab()
    {
        if (zedCamera != null)
        {

            if (tracking)
            {
                UpdateTracking();
            }

            if (videoOverlay)
            {
                zedCamera.UpdateTextures();
            }


            if (zedSVOManager != null)
            {
                if (zedSVOManager.record)
                {
                    zedCamera.Record();
                }
                if (zedSVOManager.read)
                {
                    zedSVOManager.CurrentFrame = zedCamera.GetSVOPosition();
                }
                if (zedSVOManager.read && zedSVOManager.loop)
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

    public void SetSensingMode(sl.SENSING_MODE sensingMode)
    {
        this.sensingMode = sensingMode;
    }

    void OnApplicationQuit()
    {
        if (zedCamera != null)
        {
            zedCamera.DisableTracking();
            if (zedSVOManager != null)
            {
                if (zedSVOManager.record)
                {
                    zedCamera.DisableRecording();
                }
            }
            zedCamera.Destroy();
        }
        zedCamera = null;
    }
}
