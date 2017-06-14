using System.Runtime.InteropServices;
using UnityEngine;

namespace sl
{
    public struct Resolution
    {
        public int width;
        public int height;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CameraInformations
    {
        public CalibrationParameters calibParameters;
        public CalibrationParameters calibParameters_raw;
        public uint serialNumber;
        public uint firmwareVersion;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Pose
    {
        public bool valid;
        public ulong timestap;
        public Quaternion rotation;
        public Vector3 translation;
        public int pose_confidence;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CameraParameters
    {
        /// <summary>
        /// Focal x
        /// </summary>
        public float fx;
        /// <summary>
        /// Focal y
        /// </summary>
        public float fy;
        /// <summary>
        /// Optical center x
        /// </summary>
        public float cx;
        /// <summary>
        /// Optical center y
        /// </summary>
        public float cy;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.U8, SizeConst = 5)]
        public double[] disto;

        /// <summary>
        /// Vertical field of view after stereo rectification
        /// </summary>
        public float vFOV;
        /// <summary>
        /// Horizontal field of view after stereo rectification
        /// </summary>
        public float hFOV;
        /// <summary>
        /// Diagonal field of view after stereo rectification
        /// </summary>
        public float dFOV;

        public Resolution resolution;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CalibrationParameters
    {
        /// <summary>
        /// Rotation (using Rodrigues' transformation) between the two sensors. Defined as 'tilt', 'convergence' and 'roll'
        /// </summary>
        public Vector3 Rot;

        /// <summary>
        /// Translation between the two sensors. T[0] is the distance between the two cameras in meters.
        /// </summary>
        public Vector3 Trans;
        /// <summary>
        /// Parameters of the left camera
        /// </summary>
        public CameraParameters leftCam;
        /// <summary>
        /// Parameters of the right camera
        /// </summary>
        public CameraParameters rightCam;
    };


    [StructLayout(LayoutKind.Sequential)]
    public struct Recording_state
    {
        /// <summary>
        /// status of current frame. May be true for success or false if frame could not be written in the SVO file
        /// </summary>
        public bool status;
        /// <summary>
        /// compression time for the current frame in ms
        /// </summary>
        public double current_compression_time;
        /// <summary>
        /// compression ratio (% of raw size) for the current frame
        /// </summary>
        public double current_compression_ratio;
        /// <summary>
        /// average compression time in ms since beginning of recording
        /// </summary>
        public double average_compression_time;
        /// <summary>
        /// compression ratio (% of raw size) since beginning of recording
        /// </summary>
        public double average_compression_ratio;
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct MotionPoseData
    {
        /// <summary>  
        ///boolean that indicates if tracking is activated or not. You should check that first if something wrong.
        ////// </summary>  
        public bool valid;
        /// <summary>  
        ///Timestamp of the frame that this pose estimate corresponds to.
        /// </summary>  
        public ulong timestamp;

        public Quaternion rotation;
        public Vector3 translation;
        /// <summary>  
        ///Current status of the tracking for the target frame 
        /// </summary>  
        public int frame_status;
        /// <summary>  
        ///Confidence/Quality of the pose estimation for the target frame 
        /// </summary>  
        public int pose_confidence;
    }

    public enum ZED_SELF_CALIBRATION_STATE
    {
        /// <summary>
        /// Self Calibration has not yet been called (no init called)
        /// </summary>
        SELF_CALIBRATION_NOT_CALLED,
        /// <summary>
        /// Self Calibration is currently running.
        /// </summary>
        SELF_CALIBRATION_RUNNING,
        /// <summary>
        /// Self Calibration has finished running but did not manage to get coherent values. Old Parameters are taken instead.
        /// </summary>
        SELF_CALIBRATION_FAILED,
        /// <summary>
        /// Self Calibration has finished running and did manage to get coherent values. New Parameters are taken.
        /// </summary>
        SELF_CALIBRATION_SUCCESS
    };

    public enum DEPTH_MODE
    {
        /// <summary>
        /// Disparity map not computed, only the rectified images will be available.
        /// </summary>
        NONE,
        /// <summary>
        /// Fastest mode, also requires less GPU memory, the disparity map is less robust
        /// </summary>
        PERFORMANCE,
        /// <summary>
        /// Balanced quality mode, requires less GPU memory but the disparity map is a little less detailed
        /// </summary>
        MEDIUM,
        /// <summary>
        /// Better quality mode, the disparity map is more precise
        /// </summary>
        QUALITY
    };

    public enum UNIT
    {
        MILLIMETER,
        CENTIMETER,
        METER,
        INCH,
        FOOT,
    };

    public enum TRACKING_REFERENCE
    {
        TRACKING_REFERENCE_LEFT, /*!< The left camera is taken as reference */
        TRACKING_REFERENCE_CENTER, /*!< The center of the ZED is taken as reference */
        TRACKING_REFERENCE_RIGHT /*!< The right camera is taken as reference */
    };

    public enum ERROR_CODE
    {
        /// <summary>
        ///  Every step went fine, the camera is ready to use
        /// </summary>
        SUCCESS,

        /// <summary>
        /// unsuccessful behavior
        /// </summary>
        FAILURE,
        /// <summary>
        /// No GPU found or CUDA capability of the device is not supported
        /// </summary>
        NO_GPU_COMPATIBLE,
        /// <summary>
        /// Not enough GPU memory for this depth mode, please try a faster mode (such as PERFORMANCE mode)
        /// </summary>
        NOT_ENOUGH_GPUMEM,
        /// <summary>
        /// The ZED camera is not plugged or detected 
        /// </summary>
        CAMERA_NOT_DETECTED,
        /// <summary>
        /// For Jetson only, resolution not yet supported (USB3.0 bandwidth)
        /// </summary>
        INVALID_RESOLUTION,
        /// <summary>
        /// This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue)
        /// </summary>
        LOW_USB_BANDWIDTH,
        /// <summary>
        /// ZED Settings file is not found on the host machine. Use ZED Settings tool to correct it.
        /// </summary>
        CALIBRATION_FILE_NOT_AVAILABLE,
        /// <summary>
        /// The provided SVO file is not valid
        /// </summary>
        INVALID_SVO_FILE,
        /// <summary>
        /// An recorder related error occurred (not enough free storage, invalid file)
        /// </summary>
        SVO_RECORDING_ERROR,
        /// <summary>
        ///  The requested coordinate system is not available.
        /// </summary>
        INVALID_COORDINATE_SYSTEM,
        /// <summary>
        /// The firmware of the ZED is out of date. You might install the new version.
        /// </summary>
        INVALID_FIRMWARE,
        /// <summary>
        /// in grab() only, the current call return the same frame as last call. Not a new frame.
        /// </summary>
        NOT_A_NEW_FRAME,
        /// <summary>
        /// in grab() only, a CUDA error has been detected in the process.
        /// </summary>
        CUDA_ERROR,
        /// <summary>
        /// in grab() only, ZED SDK is not initialized. Probably a missing call to sl.Camera.Init.
        /// </summary>
        CAMERA_NOT_INITIALIZED,
        /// <summary>
        /// your NVIDIA driver is too old and not compatible with your current CUDA version.
        /// </summary>
        NVIDIA_DRIVER_OUT_OF_DATE,
        /// <summary>
        /// the call of the function is not valid in the current context. Could be a missing call of sl.Camera.Init
        /// </summary>
        INVALID_FUNCTION_CALL,
        /// <summary>
        /// The SDK wasn't able to load its dependencies, the installer should be launched.
        /// </summary>
        CORRUPTED_SDK_INSTALLATION
    };

    public enum RESOLUTION
    {
        /// <summary>
        /// 2208*1242, supported frame rate : 15 fps
        /// </summary>
        HD2K,
        /// <summary>
        /// 1920*1080, supported frame rates : 15, 30 fps
        /// </summary>
        HD1080,
        /// <summary>
        /// 1280*720, supported frame rates : 15, 30, 60 fps
        /// </summary>
        HD720,
        /// <summary>
        /// 672*376, supported frame rates : 15, 30, 60, 100 fps
        /// </summary>
        VGA
    };

    public enum SENSING_MODE
    {
        /// <summary>
        /// This mode outputs ZED standard depth map that preserves edges and depth accuracy.
        /// Applications example: Obstacle detection, Automated navigation, People detection, 3D reconstruction
        /// </summary>
        STANDARD,
        /// <summary>
        /// This mode outputs a smooth and fully dense depth map.
        /// Applications example: AR/VR, Mixed-reality capture, Image post-processing
        /// </summary>
        FILL

    };

    public enum TYPE_VIEW
    {
        GET_VIEW,
        RETRIEVE_IMAGE,
        RETRIEVE_MEASURE
    }

    public enum VIEW
    {
        LEFT,
        RIGHT,
        LEFT_GREY,
        RIGHT_GREY,
        LEFT_UNRECTIFIED,
        RIGHT_UNRECTIFIED,
        LEFT_UNRECTIFIED_GREY,
        RIGHT_UNRECTIFIED_GREY,
        SIDE_BY_SIDE, /*!< Left and right image (the image width is therefore doubled) */
        DEPTH, /*!< Normalized depth image */
        CONFIDENCE, /*!< Normalized confidence image */
    };

    public enum CAMERA_SETTINGS
    {
        /// <summary>
        /// Defines the brightness control. Affected value should be between 0 and 8
        /// </summary>
        BRIGHTNESS,
        /// <summary>
        /// Defines the contrast control. Affected value should be between 0 and 8
        /// </summary>
        CONTRAST,
        /// <summary>
        /// Defines the hue control. Affected value should be between 0 and 11
        /// </summary>
        HUE,
        /// <summary>
        /// Defines the saturation control. Affected value should be between 0 and 8
        /// </summary>
        SATURATION,
        /// <summary>
        /// Defines the gain control. Affected value should be between 0 and 100 for manual control. If ZED_EXPOSURE is set to -1, the gain is in auto mode too.
        /// </summary>
        GAIN,
        /// <summary>
        /// Defines the exposure control. A -1 value enable the AutoExposure/AutoGain control. Affected value should be between 0 and 100 for manual control. A 0 value only disable auto mode without modifing the last auto values, while a 1 to 100 value disable auto mode and set exposure to chosen value
        /// </summary>
        EXPOSURE,
        /// <summary>
        /// Defines the color temperature control. Affected value should be between 2800 and 6500 with a step of 100. A value of -1 set the AWB ( auto white balance), as the boolean parameter (default) does.
        /// </summary>
        WHITEBALANCE
    };



    public enum MEASURE
    {
        /// <summary>
        /// Disparity map, 1 channel, FLOAT
        /// </summary>
        DISPARITY,
        /// <summary>
        /// Depth map, 1 channel, FLOAT
        /// </summary>
        DEPTH,
        /// <summary>
        /// Certainty/confidence of the disparity map, 1 channel, FLOAT
        /// </summary>
        CONFIDENCE,
        /// <summary>
        /// 3D coordinates of the image points, 4 channels, FLOAT (the 4th channel may contains the colors)
        /// </summary>
        XYZ,
        /// <summary>
        /// 3D coordinates and Color of the image , 4 channels, FLOAT (the 4th channel encode 4 UCHAR for color in R-G-B-A order)
        /// </summary>
        XYZRGBA,
        /// <summary>
        /// 3D coordinates and Color of the image , 4 channels, FLOAT (the 4th channel encode 4 UCHAR for color in B-G-R-A order)
        /// </summary>
        XYZBGRA,
        /// <summary>
        /// 3D coordinates and Color of the image , 4 channels, FLOAT (the 4th channel encode 4 UCHAR for color in A-R-G-B order)
        /// </summary>
        XYZARGB,
        /// <summary>
        /// 3D coordinates and Color of the image , 4 channels, FLOAT (the 4th channel encode 4 UCHAR for color in A-B-G-R order)
        /// </summary>
        XYZABGR
    };

    /// <summary>
    /// Only few functions of tracking use this system, the path is the default value
    /// </summary>
    public enum REFERENCE_FRAME
    {
        /// <summary>
        /// The matrix contains the displacement from the first camera to the current one
        /// </summary>
        WORLD,
        /// <summary>
        /// The matrix contains the displacement from the previous camera position to the current one
        /// </summary>
        CAMERA
    };

    public enum TRACKING_FRAME_STATE
    {
        /// <summary>
        /// The tracking is searching a match from the database to relocate at a previously known position
        /// </summary>
        TRACKING_SEARCH,
        /// <summary>
        /// The tracking operates normally, the path should be correct
        /// </summary>
        TRACKING_OK,
        /// <summary>
        /// The tracking is not enabled
        /// </summary>
        TRACKING_OFF
    }

    public enum SVO_COMPRESSION_MODE
    {
        /// <summary>
        /// RAW images, no compression
        /// </summary>
        RAW_BASED,
        /// <summary>
        /// new Lossless, with png/zstd based compression : avg size = 42% of RAW
        /// </summary>
        LOSSLESS_BASED,
        /// <summary>
        /// new Lossy, with jpeg based compression : avg size = 22% of RAW
        /// </summary>
        LOSSY_BASED
    }
}