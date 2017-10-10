using UnityEngine;
using System;

public class ZEDManagerVR : MonoBehaviour
{
    /// <summary>
    /// Current instance of the ZED Camera
    /// </summary>
    private sl.ZEDCamera zedCamera;


    [Header("Camera")]
    /// <summary>
    /// Enable grab threading
    /// </summary>
    [HideInInspector]
    [Tooltip("Thread grab calls. May increase performance for multi-core processors.")]
    public bool multithreading = false;

    [Header("Motion Tracking")]

    /// <summary>
    /// Enable positional tracking
    /// </summary>
    public bool trackingTranslation = true;
    /// <summary>
    /// Offset of the tracking reference, if the ZED is set to the front of the Headset.
    /// </summary>
    public Vector3 cameraHeadOffset = new Vector3(0.06f, -0.12f, -0.15f);

    private Quaternion orientation;
    private Vector3 position;

    private bool tracking = true;
    private bool isThreaded = false;



    Vector3 destination = new Vector3();
    Vector3 start = new Vector3();
    Vector3 previousPose = new Vector3();

    Vector3 distance = new Vector3();
    int numberTick = 0;
    private float magnitudeAcc = 0.009f;
    private Quaternion previousQuaternion;
    private Vector3 posNulle = new Vector3();
    private Vector3 cumul = new Vector3();
    private Vector3 acceleration = new Vector3();

    private SVOManager ZedSVOManager;
    private sl.RESOLUTION resolution = sl.RESOLUTION.VGA;
    private bool headset = false;
    void Awake()
    {
        position = new Vector3(0, 0.0f, 0);
        orientation = Quaternion.identity;

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
                zedCamera.CreateCameraLive(resolution, 0.0f);
            }
        }
        else
        {
            zedCamera.CreateCameraLive(resolution, 0.0f);
        }
        Debug.Log("ZED SDK Version " + sl.ZEDCamera.GetSDKVersion());

        //Initialize the camera in performance mode.
        sl.ERROR_CODE e = zedCamera.Init(sl.DEPTH_MODE.PERFORMANCE);

        if (e != sl.ERROR_CODE.SUCCESS)
        {
            throw new Exception("Initialization failed " + e.ToString());
        }


        ZEDUpdater.GetInstance().SetMultiThread(isThreaded);

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
        if (UnityEngine.VR.VRSettings.enabled)
        {
            headset = true;
            UnityEngine.VR.InputTracking.Recenter();
        }
        else
        {
            headset = false;
            cameraHeadOffset.y = 0;
            cameraHeadOffset.z = 0;

            Debug.LogWarning("This script needs the VR to work correctly");
        }
    }

    private void Start()
    {

        //Delays the enable tracking to get the position of the HMD
        if (tracking)
        {
#if UNITY_5_5_OR_NEWER
            if (UnityEngine.VR.VRSettings.isDeviceActive)
            {
                Quaternion quat = UnityEngine.VR.InputTracking.GetLocalRotation(UnityEngine.VR.VRNode.Head);
                Vector3 vec = new Vector3(0, 0, 0);
                if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, true) == sl.ERROR_CODE.SUCCESS)))
                {
                    throw new Exception("Error, tracking not available");
                }
            }
            else
            {
                Quaternion quat = Quaternion.identity;
                Vector3 vec = new Vector3(0, 0, 0);
                if (!(tracking = (zedCamera.EnableTracking(ref quat, ref vec, true) == sl.ERROR_CODE.SUCCESS)))
                {
                    throw new Exception("Error, tracking not available");
                }
            }
           
#else
                Quaternion quat = UnityEngine.VR.InputTracking.GetLocalRotation(UnityEngine.VR.VRNode.Head);
                Vector3 vec = new Vector3(0, 0, 0);
                if (!(tracking = zedCamera.EnableTracking(ref quat, ref vec, false)))
                {
                    throw new Exception("Error, tracking not available");
                }
#endif
        }


    }

    

    /// <summary>
    /// Get the tracking position from the ZED and update the manager's position
    /// If the tracking of the ZED and the HMD have shifted, put an offset on rotation
    /// </summary>
    private void UpdateTracking()
    {

        Quaternion diffRotation = Quaternion.identity;
        Quaternion rotationHMD = Quaternion.identity;

        Quaternion orientationPose = Quaternion.identity;
        //Get the pose with an offset
        Vector3 world_pose_translation = new Vector3();
        Vector3 camera_pose_translation = new Vector3();
        // Get motion tracking in camera reference frame (current position regarding previous camera position)
        zedCamera.GetPosition(ref orientationPose, ref position, sl.REFERENCE_FRAME.CAMERA);
        // Get motion tracking in a target frame that may be different from the camera reference frame.
        zedCamera.GetPosition(ref orientationPose, ref camera_pose_translation, ref diffRotation, ref posNulle, sl.REFERENCE_FRAME.CAMERA);
        // Get motion tracking in world reference frame.
        zedCamera.GetPosition(ref orientation, ref world_pose_translation, sl.REFERENCE_FRAME.WORLD);
        if (headset)
        {
            rotationHMD = UnityEngine.VR.InputTracking.GetLocalRotation(UnityEngine.VR.VRNode.Head);
        }
        else
        {
            rotationHMD = orientation;
        }
        if (trackingTranslation)
        {
            cumul += rotationHMD * position;
        }
        if (Mathf.Abs(position.y - camera_pose_translation.y) > 0.05)
        {
            Vector3 correctionPos = position;
            correctionPos.x = 0;
            correctionPos.z = 0;
            destination = cumul;
            destination.y = world_pose_translation.y;
            start = cumul;
            distance = (destination - start);
            numberTick = 3 * (int)(1.0f / Time.deltaTime);
        }
        if (trackingTranslation)
        {
            acceleration = new Vector3();
            acceleration = position - previousPose;
            acceleration.y = 0;
            Quaternion d = previousQuaternion * Quaternion.Inverse(rotationHMD);
            Vector3 diffRotationHMD = new Vector4(d.x, d.y, d.z);
            if (numberTick > 0)
            {
                if (acceleration.magnitude >= magnitudeAcc || diffRotationHMD.magnitude > 0.008)
                {
                    cumul += distance / numberTick;
                    start = cumul;
                    destination.x = cumul.x;
                    destination.z = cumul.z;
                    distance = (destination - start);
                    numberTick--;
                }
            }
        }
        previousPose = position;
        previousQuaternion = rotationHMD;
        Vector3 transformV = cumul;
        Quaternion transformQ = orientation;
        sl.ZEDCamera.TransformPose(ref transformQ, ref transformV, ref diffRotation, ref cameraHeadOffset);
        transform.localPosition = transformV;
        if (!headset)
        {
            transform.GetChild(0).localRotation = rotationHMD;
        }
    }

    public void OnEnable()
    {
        ZEDUpdater.OnGrab += OnGrab;
    }

    public void OnDisable()
    {
        ZEDUpdater.OnGrab -= OnGrab;
    }
    void OnGrab()
    {
        if (zedCamera != null)
        {

            UpdateTracking();
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
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        ZEDUpdater.GetInstance().Update(sl.SENSING_MODE.STANDARD, true);
    }



    void OnApplicationQuit()
    {
        if (zedCamera != null)
        {
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
