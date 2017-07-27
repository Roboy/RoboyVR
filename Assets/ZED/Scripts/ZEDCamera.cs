using UnityEngine;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine.Rendering;

namespace sl
{
    /*
     * Unity representation of a sl::Camera
     */
    public class ZEDCamera
    {
        /// <summary>
        /// Informations of requested textures
        /// </summary>
        private struct TextureRequested
        {
            public int type;
            public int option;
        };

        /********* Camera members ********/

        //DLL name
        const string nameDll = "sl_unitywrapper";

        //List of all textures using the type as unique key
        private Dictionary<int, Dictionary<int, Texture2D>> textures;

        //List of all requested textures 
        private List<TextureRequested> texturesRequested;

        //Width of the textures
        private int imageWidth;

        //Height of the textures
        private int imageHeight;

        //True if the camera is ready
        private bool cameraIsReady = false;

        //ZED projection matrix
        private Matrix4x4 projection = new Matrix4x4();

        //Singleton of ZEDCamera
        private static ZEDCamera instance = null;

        //True if the SDK is installed
        private static bool pluginIsReady = true;

        //Mutex for threaded rendering
        private static object _lock = new object();

        //The current resolution
        private RESOLUTION currentResolution;

        //Callback for c++ debug, should not be used in C#
        private delegate void DebugCallback(string message);

        //HD720 default FPS
        private uint fpsMax = 60;
        private ZEDCameraSettingsManager cameraSettingsManager = new ZEDCameraSettingsManager();



        // Baseline of the camera
        private float baseline = 0.0f;
        public float Baseline
        {
            get { return baseline; }
        }


        static private string[] dependenciesNeeded =
        {
            "sl_calibration64.dll",
            "sl_core64.dll",
            "sl_depthcore64.dll",
            "sl_disparityFusion64.dll",
            "sl_scanning64.dll",
            "sl_svorw64.dll",
            "sl_zed64.dll"
        };
        const int tagZEDCamera = 20;
        public static int Tag
        {
            get { return tagZEDCamera; }
        }

        const int tagOneObject = 12;
        public static int TagOneObject
        {
            get { return tagOneObject; }
        }

        /******** DLL members ***********/

        [DllImport(nameDll, EntryPoint = "GetRenderEventFunc")]
        private static extern IntPtr GetRenderEventFunc();

        [DllImport(nameDll, EntryPoint = "dllz_register_callback_debuger")]
        private static extern void dllz_register_callback_debuger(DebugCallback callback);


        /*
         * Create functions 
         */
        [DllImport(nameDll, EntryPoint = "dllz_create_camera_live")]
        private static extern void dllz_create_camera_live(int mode = (int)RESOLUTION.HD1080, float fps = 0.0f, int linux_id = 0);

        [DllImport(nameDll, EntryPoint = "dllz_create_camera_svo")]
        private static extern void dllz_create_camera_svo(byte[] svo_path);



        /*
        * Opening function (Open camera and create textures)
        */
        [DllImport(nameDll, EntryPoint = "dllz_open")]
        private static extern int dllz_open(int mode, float minDist, bool disable, bool vflip);


        /*
         * Close function
         */
        [DllImport(nameDll, EntryPoint = "dllz_close")]
        private static extern void dllz_close();


        /*
        * Grab function
        */
        [DllImport(nameDll, EntryPoint = "dllz_grab")]
        private static extern int dllz_grab(int sensingMode, int depth, int measureMode);


        /*
        * Recording functions
        */
        [DllImport(nameDll, EntryPoint = "dllz_enable_recording")]
        private static extern int dllz_enable_recording(byte[] video_filename, int compresssionMode);

        [DllImport(nameDll, EntryPoint = "dllz_record")]
        private static extern void dllz_record(ref Recording_state state);

        [DllImport(nameDll, EntryPoint = "dllz_disable_recording")]
        private static extern bool dllz_disable_recording();


        /*
        * Texturing functions
        */
        [DllImport(nameDll, EntryPoint = "dllz_update_textures")]
        private static extern void dllz_update_textures();

        [DllImport(nameDll, EntryPoint = "dllz_register_texture_image_type")]
        private static extern int dllz_register_texture_image_type(int option, IntPtr id);

        [DllImport(nameDll, EntryPoint = "dllz_register_texture_measure_type")]
        private static extern int dllz_register_texture_measure_type(int option, IntPtr id);


        /*
        * Self-calibration function
        */
        [DllImport(nameDll, EntryPoint = "dllz_reset_self_calibration")]
        private static extern void dllz_reset_self_calibration();

        [DllImport(nameDll, EntryPoint = "dllz_get_self_calibration_state")]
        private static extern int dllz_get_self_calibration_state();


        /*
         * Camera control functions
         */
        [DllImport(nameDll, EntryPoint = "dllz_get_zed_firmware")]
        private static extern int dllz_get_zed_firmware();

        [DllImport(nameDll, EntryPoint = "dllz_get_zed_serial")]
        private static extern int dllz_get_zed_serial();

        [DllImport(nameDll, EntryPoint = "dllz_set_camera_fps")]
        private static extern void dllz_set_camera_fps(int fps);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_fps")]
        private static extern float dllz_get_camera_fps();

        [DllImport(nameDll, EntryPoint = "dllz_get_width")]
        private static extern int dllz_get_width();

        [DllImport(nameDll, EntryPoint = "dllz_get_height")]
        private static extern int dllz_get_height();

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_information")]
        private static extern IntPtr dllz_get_camera_information();


        [DllImport(nameDll, EntryPoint = "dllz_set_camera_settings")]
        private static extern void dllz_set_camera_settings(int mode, int value, int usedefault);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_settings")]
        private static extern int dllz_get_camera_settings(int mode);

        [DllImport(nameDll, EntryPoint = "dllz_is_zed_connected")]
        private static extern int dllz_is_zed_connected();


        [DllImport(nameDll, EntryPoint = "dllz_get_camera_timestamp")]
        private static extern ulong dllz_get_camera_timestamp();

        [DllImport(nameDll, EntryPoint = "dllz_get_current_timestamp")]
        private static extern ulong dllz_get_current_timestamp();

        [DllImport(nameDll, EntryPoint = "dllz_get_image_updater_time_stamp")]
        private static extern ulong dllz_get_image_updater_time_stamp();


        [DllImport(nameDll, EntryPoint = "dllz_get_frame_dropped_count")]
        private static extern uint dllz_get_frame_dropped_count();


        /*
         * SVO control functions
         */

        [DllImport(nameDll, EntryPoint = "dllz_set_svo_position")]
        private static extern void dllz_set_svo_position(int frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_svo_number_of_frames")]
        private static extern int dllz_get_svo_number_of_frames();

        [DllImport(nameDll, EntryPoint = "dllz_get_svo_position")]
        private static extern int dllz_get_svo_position();


        /*
         * Depth Sensing utils functions
         */
        [DllImport(nameDll, EntryPoint = "dllz_set_confidence_threshold")]
        private static extern void dllz_set_confidence_threshold(int threshold);

        [DllImport(nameDll, EntryPoint = "dllz_get_confidence_threshold")]
        private static extern int dllz_get_confidence_threshold();

        [DllImport(nameDll, EntryPoint = "dllz_set_depth_max_range_value")]
        private static extern void dllz_set_depth_max_range_value(float distanceMax);

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_max_range_value")]
        private static extern float dllz_get_depth_max_range_value();

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_value")]
        private static extern float dllz_get_depth_value(uint x, uint y);

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_min_range_value")]
        private static extern float dllz_get_depth_min_range_value();



        /*
         * Motion Tracking functions
         */
        [DllImport(nameDll, EntryPoint = "dllz_enable_tracking")]
        private static extern int dllz_enable_tracking(ref Quaternion quat, ref Vector3 vec, bool enableSpatialMemory = false, string aeraFilePath = "");

        [DllImport(nameDll, EntryPoint = "dllz_disable_tracking")]
        private static extern void dllz_disable_tracking(byte[] path);

        [DllImport(nameDll, EntryPoint = "dllz_get_position_data")]
        private static extern int dllz_get_position_data(ref Pose pose, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_position")]
        private static extern int dllz_get_position(ref Quaternion quat, ref Vector3 vec, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_position_at_target_frame")]

        private static extern int dllz_get_position_at_target_frame(ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_transform_pose")]
        private static extern void dllz_transform_pose(ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation);


        /*
         * Specific plugin functions
         */
        [DllImport(nameDll, EntryPoint = "dllz_check_plugin")]
        private static extern void dllz_check_plugin();

        [DllImport(nameDll, EntryPoint = "dllz_set_is_threaded")]
        private static extern void dllz_set_is_threaded();

        [DllImport(nameDll, EntryPoint = "dllz_get_sdk_version")]
        private static extern IntPtr dllz_get_sdk_version();

        [DllImport(nameDll, EntryPoint = "dllz_compute_offset")]
        private static extern void dllz_compute_offset(float[] A, float[] B, int nbVectors, float[] C);

        public static void ComputeOffset(float[] A, float[] B, int nbVectors, ref Quaternion rotation, ref Vector3 translation)
        {
            float[] C = new float[16];
            if (A.Length != 4 * nbVectors || B.Length != 4 * nbVectors || C.Length != 16) return;
            dllz_compute_offset(A, B, nbVectors, C);

            Matrix4x4 m = Matrix4x4.identity;
            Float2Matrix(ref m, C);

            rotation = Matrix4ToQuaternion(m);
            Vector4 t = m.GetColumn(3);
            translation.x = t.x;
            translation.y = t.y;
            translation.z = t.z;

        }

        /// <summary>
        /// Return a string from a pointer to char
        /// </summary>
        /// <param name="ptr"></param>
        /// <returns>the string</returns>
        private static string PtrToStringUtf8(IntPtr ptr)
        {
            if (ptr == IntPtr.Zero)
            {
                return "";
            }
            int len = 0;
            while (Marshal.ReadByte(ptr, len) != 0)
                len++;
            if (len == 0)
            {
                return "";
            }
            byte[] array = new byte[len];
            Marshal.Copy(ptr, array, 0, len);
            return System.Text.Encoding.ASCII.GetString(array);
        }


        /// <summary>
        /// Display a console message from c++
        /// </summary>
        /// <param name="message"></param>
        private static void DebugMethod(string message)
        {
            Debug.Log("[ZED plugin]: " + message);
        }

        /// <summary>
        /// Convert a pointer to char to an array of bytes
        /// </summary>
        /// <param name="ptr"></param>
        /// <returns>The array</returns>
        private static byte[] StringUtf8ToByte(string str)
        {
            byte[] array = System.Text.Encoding.ASCII.GetBytes(str);
            return array;
        }


        /// <summary>
        /// Get the max fps for each resolution, higher fps will cause lower GPU performance
        /// </summary>
        /// <param name="reso"></param>
        /// <returns>The resolution</returns>
        static private uint GetFpsForResolution(RESOLUTION reso)
        {
            if (reso == RESOLUTION.HD1080) return 30;
            else if (reso == RESOLUTION.HD2K) return 15;
            else if (reso == RESOLUTION.HD720) return 60;
            else if (reso == RESOLUTION.VGA) return 100;
            return 30;
        }

        /// <summary>
        /// Get a quaternion from a matrix with a minimum size of 3x3
        /// </summary>
        /// <param name="m">The matrix </param>
        /// <returns>A quaternion which contains the rotation</returns>
        public static Quaternion Matrix4ToQuaternion(Matrix4x4 m)
        {
            Quaternion q = new Quaternion();
            q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
            q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
            q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
            q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
            q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
            q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
            q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
            return q;
        }

        public static void TransformPose(ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation)
        {
            dllz_transform_pose(ref quaternion, ref translation, ref targetQuaternion, ref targetTranslation);
        }
        /// <summary>
        /// Check if the plugin is available
        /// </summary>
        public static string CheckPlugin()
        {
            try
            {
                dllz_check_plugin();
            }
            catch (DllNotFoundException)
            {
                pluginIsReady = false;
                string env = Environment.GetEnvironmentVariable("ZED_SDK_ROOT_DIR");
                if (env != null)
                {
                    bool error = CheckDependencies(System.IO.Directory.GetFiles(env + "\\bin"));
                    if (!error)
                    {
                        Debug.LogError("[ZED Plugin ] : The version of this SDK may not match the version used by the plugin, or another dependency is missing");
                    }
                }
                else
                {
                    Debug.LogError("[ ZED Plugin ] : The SDK is not installed");
                }
                string errorMessage = "The ZED plugin could not be loaded. Please check you have the ZED SDK and its dependencies (CUDA) installed." +
                                    "\n Unity needs to be restarted after installation of the missing dependencies." +
                                    "\n If the problem persists, please contact our support team at support@stereolabs.com\n";
                return errorMessage;

            }
            return "";
        }

        static private bool CheckDependencies(string[] filesFound)
        {
            bool isASDKPb = false;
            if (filesFound == null) return true;
            foreach (string dependency in dependenciesNeeded)
            {
                bool found = false;
                foreach (string file in filesFound)
                {
                    if (System.IO.Path.GetFileName(file).Equals(dependency))
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    isASDKPb = true;
                    Debug.LogError("[ZED Plugin ] : " + dependency + " is not found");
                }
            }
            return isASDKPb;
        }



        /// <summary>
        /// Gets an instance of the ZEDCamera
        /// </summary>
        /// <returns>The instance</returns>
        public static ZEDCamera GetInstance()
        {
            lock (_lock)
            {
                if (instance == null)
                {
                    instance = new ZEDCamera();
                    CheckPlugin();
                    dllz_register_callback_debuger(new DebugCallback(DebugMethod));
                }
                return instance;
            }
        }


        /// <summary>
        /// Private constructor
        /// </summary>
        private ZEDCamera()
        {
            //Create the textures
            textures = new Dictionary<int, Dictionary<int, Texture2D>>();
            texturesRequested = new List<TextureRequested>();

        }

        /// <summary>
        /// Create a camera in live mode
        /// </summary>
        /// <param name="mode"></param>
        /// <param name="fps"></param>
        /// <param name="linux_id"></param>
        public void CreateCameraLive(RESOLUTION mode = RESOLUTION.HD720, float fps = 0.0f, int linux_id = 0)
        {
            string infoSystem = SystemInfo.graphicsDeviceType.ToString().ToUpper();
            if (!infoSystem.Equals("DIRECT3D11"))
            {
                throw new Exception("The graphic library [" + infoSystem + "] is not supported");
            }

            currentResolution = mode;
            fpsMax = GetFpsForResolution(mode);
            if (fps == 0.0f) fps = fpsMax;
            ZEDUpdater.GetInstance().fpsMax = fpsMax;
            dllz_create_camera_live((int)mode, fps, linux_id);
        }

        /// <summary>
        /// Create a camera in svo mode
        /// </summary>
        /// <param name="svoPath"></param>
        public void CreateCameraSVO(string svoPath)
        {
            string infoSystem = SystemInfo.graphicsDeviceType.ToString().ToUpper();
            if (!infoSystem.Equals("DIRECT3D11"))
            {
                throw new Exception("The graphic library [" + infoSystem + "] is not supported");
            }
            dllz_create_camera_svo(StringUtf8ToByte(svoPath));
        }

        /// <summary>
        /// Close the camera and delete all textures
        /// Once destroyed, you need to recreate a camera to restart again
        /// </summary>
        public void Destroy()
        {
            ZEDUpdater.GetInstance().Destroy();

            cameraIsReady = false;

            dllz_close();

            DestroyAllTexture();
            instance = null;
        }

        /// <summary>
        /// The Init function must be called after the instantiation. The function checks if the ZED camera is plugged and opens it, initialize the projection matix and command buffers to update textures
        /// </summary>
        /// <param name="mode_">defines the quality of the depth map, affects the level of details and also the computation time.</param>
        /// <param name="minDist_">specify the minimum depth information that will be computed, in the unit you previously define.</param>
        /// <param name="disable">if set to true, it will disable self-calibration and take the initial calibration parameters without optimizing them</param>
        /// <returns>ERROR_CODE : The error code gives information about the
        /// internal process, if SUCCESS is returned, the camera is ready to use.
        /// Every other code indicates an error and the program should be stopped.
        /// 
        /// For more details see sl::zed::ERRCODE.</returns>
        public ERROR_CODE Init(DEPTH_MODE mode_ = DEPTH_MODE.PERFORMANCE, float minDist_ = -1, bool disable = false)
        {
            int v = dllz_open((int)mode_, minDist_, disable, false);
            if ((ERROR_CODE)v != ERROR_CODE.SUCCESS)
            {
                cameraIsReady = false;
                throw new Exception("Error initialization camera, no zed available [" + ((ERROR_CODE)v).ToString() + "]");
            }
            cameraIsReady = true;
            imageWidth = dllz_get_width();
            imageHeight = dllz_get_height();
            FillProjectionMatrix();
            baseline = GetCameraInformation().calibParameters.Trans[0];

            return ((ERROR_CODE)v);
        }



        /// <summary>
        /// Fill the projection matrix with the parameters of the ZED, needs to be called only once. This projection matrix is off center.
        /// </summary>
        /// <param name="zFar"></param>
        /// <param name="zNear"></param>
        public void FillProjectionMatrix(float zFar = 500, float zNear = 0.2f)
        {
            CalibrationParameters parameters = GetCameraInformation().calibParameters;
            float fovx = parameters.leftCam.hFOV * Mathf.Deg2Rad;
            float fovy = parameters.leftCam.vFOV * Mathf.Deg2Rad;

            projection[0, 0] = 1.0f / Mathf.Tan(fovx * 0.5f);
            projection[0, 1] = 0;
            projection[0, 2] = 2 * ((ImageWidth - 1 * parameters.leftCam.cx) / ImageWidth) - 1.0f;
            projection[0, 3] = 0;

            projection[1, 0] = 0;
            projection[1, 1] = 1.0f / Mathf.Tan(fovy * 0.5f);
            projection[1, 2] = -(2 * ((ImageHeight - 1 * parameters.leftCam.cy) / ImageHeight) - 1.0f);
            projection[1, 3] = 0;

            projection[2, 0] = 0;
            projection[2, 1] = 0;
            projection[2, 2] = -(zFar + zNear) / (zFar - zNear);
            projection[2, 3] = -(2.0f * zFar * zNear) / (zFar - zNear);

            projection[3, 0] = 0;
            projection[3, 1] = 0;
            projection[3, 2] = -1;
            projection[3, 3] = 0.0f;


        }


        /// <summary>
        /// Grab a new image, rectifies it and computes the
        /// disparity map and optionally the depth map.
        /// The grabbing function is typically called in the main loop.
        /// </summary>
        /// <param name="sensingMode">defines the type of disparity map, more info : SENSING_MODE definition</param>
        /// <returns>the function returns false if no problem was encountered,
        /// true otherwise.</returns>

        public sl.ERROR_CODE Grab(SENSING_MODE sensingMode = SENSING_MODE.FILL, bool computeDepth = true, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.CAMERA)
        {
            AssertCameraIsReady();
            sl.ERROR_CODE error = sl.ERROR_CODE.FAILURE;
            error = (sl.ERROR_CODE)dllz_grab((int)sensingMode, Convert.ToInt32(computeDepth), (int)referenceFrame);
            return error;
        }

        /// <summary>
        ///  The reset function can be called at any time AFTER the Init function has been called.
        ///  It will reset and calculate again correction for misalignment, convergence and color mismatch.
        ///  It can be called after changing camera parameters without needing to restart your executable.
        /// </summary>
        ///
        /// <returns>ERRCODE : error boolean value : the function returns false if no problem was encountered,
        /// true otherwise.
        /// if no problem was encountered, the camera will use new parameters. Otherwise, it will be the old ones
        ///</returns>
        public void ResetSelfCalibration()
        {
            AssertCameraIsReady();
            dllz_reset_self_calibration();
        }

        /// <summary>
        /// Creates a file for recording the current frames.
        /// </summary>
        /// <param name="videoFileName">can be a *.svo file or a *.avi file (detected by the suffix name provided)</param>
        /// <param name="compressionMode">can be one of the sl.SVO_COMPRESSION_MODE enum</param>
        /// <returns>an sl.ERRCODE that defines if file was successfully created and can be filled with images</returns>
        public ERROR_CODE EnableRecording(string videoFileName, SVO_COMPRESSION_MODE compressionMode = SVO_COMPRESSION_MODE.LOSSLESS_BASED)
        {
            AssertCameraIsReady();
            return (ERROR_CODE)dllz_enable_recording(StringUtf8ToByte(videoFileName), (int)compressionMode);
        }

        /// <summary>
        /// Record the images, EnableRecording needs to be called before.
        /// </summary>
        public Recording_state Record()
        {
            Recording_state state = new Recording_state();
            dllz_record(ref state);
            return state;
        }

        /// <summary>
        /// Stops the recording and closes the file.
        /// </summary>
        /// <returns></returns>
        public bool DisableRecording()
        {
            AssertCameraIsReady();
            return dllz_disable_recording();
        }

        /// <summary>
        /// Set a new frame rate for the camera, or the closest available frame rate.
        /// </summary>
        /// <param name="fps"></param>
        /// <returns></returns>
        public void SetCameraFPS(int fps)
        {
            if (GetFpsForResolution(currentResolution) >= fps)
            {
                fpsMax = (uint)fps;
                ZEDUpdater.GetInstance().fpsMax = fpsMax;

            }

            AssertCameraIsReady();
            dllz_set_camera_fps(fps);
        }

        /// <summary>
        /// Sets the position of the SVO file to a desired frame.
        /// </summary>
        /// <param name="frame"> the number of the desired frame to be decoded.</param>
        /// <returns></returns>
        public void SetSVOPosition(int frame)
        {
            AssertCameraIsReady();
            dllz_set_svo_position(frame);
        }

        /// <summary>
        /// Gets the current confidence threshold value for the disparity map (and by extension the depth map).
        /// </summary>
        /// <returns>current filtering value between 0 and 100.</returns>
        public int GetConfidenceThreshold()
        {
            AssertCameraIsReady();
            return dllz_get_confidence_threshold();
        }
        /// <summary>
        /// Get the time stamp at the time the frame has been extracted from USB stream. (should be called after a grab())
        /// </summary>
        /// <returns>Current time stamp in ns. -1 is not available(SVO file without compression). 
        /// Note that new SVO file from SDK 1.0.0 (with compression) contains the camera time stamp for each frame.</returns>
        public ulong GetCameraTimeStamp()
        {
            AssertCameraIsReady();
            return dllz_get_camera_timestamp();
        }

        /// <summary>
        /// Get the current time stamp at the time the function is called. Can be compared to the camera time stamp for synchronization. 
        /// Use this function to compare the current time stamp and the camera time stamp, since they have the same reference (Computer start time).
        /// </summary>
        /// <returns>The timestamp</returns>
        public ulong GetCurrentTimeStamp()
        {
            AssertCameraIsReady();
            return dllz_get_current_timestamp();
        }

        /// <summary>
        /// Last time stamp from image update, (based on the computer time stamp)
        /// </summary>
        /// <returns>The timestamp</returns>
        public ulong GetImageUpdaterTimeStamp()
        {
            return dllz_get_image_updater_time_stamp();
        }

        /// <summary>
        /// Get the current position of the SVO in the record
        /// </summary>
        /// <returns>The position</returns>
        public int GetSVOPosition()
        {
            AssertCameraIsReady();
            return dllz_get_svo_position();
        }

        /// <summary>
        /// Get the number of frames in the SVO file.
        /// </summary>
        /// <returns>SVO Style Only : the total number of frames in the SVO file(-1 if the SDK is not reading a SVO)</returns>
        public int GetSVONumberOfFrames()
        {
            AssertCameraIsReady();
            return dllz_get_svo_number_of_frames();
        }

        /// <summary>
        /// Get the closest measurable distance by the camera, according to the camera and the depth map parameters.
        /// </summary>
        /// <returns>The closest depth</returns>
        public float GetDepthMinRangeValue()
        {
            AssertCameraIsReady();
            return dllz_get_depth_min_range_value();
        }

        /// <summary>
        /// Returns the current maximum distance of depth/disparity estimation.
        /// </summary>
        /// <returns>The closest depth</returns>
        public float GetDepthMaxRangeValue()
        {
            AssertCameraIsReady();
            return dllz_get_depth_max_range_value();
        }

        /// <summary>
        /// Initialize and Start the tracking functions
        /// </summary>
        /// <param name="quat"> rotation used as initial world transform.By default it should be identity.</param>
        /// <param name="vec"> translation used as initial world transform. By default it should be identity.</param>
        /// <param name="enableSpatialMemory">  (optional) define if spatial memory is enable or not.</param>
        /// <param name="areaFilePath"> (optional) file of spatial memory file that has to be loaded to relocate in the scene.</param>
        /// <returns></returns>

        public sl.ERROR_CODE EnableTracking(ref Quaternion quat, ref Vector3 vec, bool enableSpatialMemory = true, string areaFilePath = "")
        {
            AssertCameraIsReady();
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            lock (ZEDUpdater.GetInstance().grabLock)
            {
                trackingStatus = (sl.ERROR_CODE)dllz_enable_tracking(ref quat, ref vec, enableSpatialMemory, areaFilePath);
            }
            return trackingStatus;
        }


        /// <summary>
        ///  Stop the motion tracking, if you want to restart, call enableTracking().
        /// </summary>
        /// <param name="path">The path to save the area file</param>
        public void DisableTracking(string path = "")
        {
            AssertCameraIsReady();
            dllz_disable_tracking(StringUtf8ToByte(path));
        }

        /// <summary>
        /// Register a texture to the base
        /// </summary>
        private void RegisterTexture(Texture2D m_Texture, int type, int mode)
        {
            TextureRequested t = new TextureRequested();

            t.type = type;
            t.option = mode;
            texturesRequested.Add(t);
            textures[type].Add(mode, m_Texture);
        }



        /// <summary>
        /// Create or retrieve a texture of type Image (from Camera::retrieveImage()). Create the texture if needed.
        /// </summary>
        /// <param name="mode"></param>
        /// <returns></returns>
        public Texture2D CreateTextureImageType(VIEW mode)
        {
            if (HasTexture((int)TYPE_VIEW.RETRIEVE_IMAGE, (int)mode))
            {
                return textures[(int)TYPE_VIEW.RETRIEVE_IMAGE][(int)mode];
            }
            if (!cameraIsReady)
                return null;

            Texture2D m_Texture;
            if (mode == VIEW.LEFT_GREY || mode == VIEW.RIGHT_GREY || mode == VIEW.LEFT_UNRECTIFIED_GREY || mode == VIEW.RIGHT_UNRECTIFIED_GREY)
            {
                m_Texture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.Alpha8, false);

            }
            else if (mode == VIEW.SIDE_BY_SIDE)
            {
                m_Texture = new Texture2D(ImageWidth * 2, ImageHeight, TextureFormat.RGBA32, false);
            }
            else
            {
                m_Texture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGBA32, false);
            }
            m_Texture.filterMode = FilterMode.Trilinear;
            //m_Texture.anisoLevel = 1;
            m_Texture.Apply();

            IntPtr idTexture = m_Texture.GetNativeTexturePtr();
            int error = dllz_register_texture_image_type((int)mode, idTexture);
            if (error != 0)
            {
                throw new Exception("Error Cuda " + error + " if the problem appears again, please contact the support");
            }
            if (!textures.ContainsKey((int)TYPE_VIEW.RETRIEVE_IMAGE))
            {
                textures.Add((int)TYPE_VIEW.RETRIEVE_IMAGE, new Dictionary<int, Texture2D>());
            }
            RegisterTexture(m_Texture, (int)TYPE_VIEW.RETRIEVE_IMAGE, (int)mode);

            return m_Texture;
        }

        /// <summary>
        /// Create or retrieve a texture of type Measure (from Camera::retrieveMeasure()). Create the texture if needed.
        /// </summary>
        /// <param name="mode"></param>
        /// <returns></returns>
        public Texture2D CreateTextureMeasureType(MEASURE mode)
        {
            if (HasTexture((int)TYPE_VIEW.RETRIEVE_MEASURE, (int)mode))
            {
                return textures[(int)TYPE_VIEW.RETRIEVE_MEASURE][(int)mode];
            }
            if (!cameraIsReady)
                return null;

            Texture2D m_Texture;

            if (mode == MEASURE.XYZ || mode == MEASURE.XYZABGR || mode == MEASURE.XYZARGB || mode == MEASURE.XYZBGRA || mode == MEASURE.XYZRGBA || mode == MEASURE.NORMALS)
            {
                m_Texture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGBAFloat, false, true);
            }
            else if (mode == MEASURE.DEPTH || mode == MEASURE.CONFIDENCE || mode == MEASURE.DISPARITY)
            {
                m_Texture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RFloat, false, true);
            }
            else m_Texture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGBA32, false, true);
            m_Texture.filterMode = FilterMode.Point;

            m_Texture.Apply();

            IntPtr idTexture = m_Texture.GetNativeTexturePtr();

            int error = dllz_register_texture_measure_type((int)mode, idTexture);

            if (error != 0)
            {
                throw new Exception("Error Cuda " + error + " if the problem appears again, please contact the support");
            }
            if (!textures.ContainsKey((int)TYPE_VIEW.RETRIEVE_MEASURE))
            {
                textures.Add((int)TYPE_VIEW.RETRIEVE_MEASURE, new Dictionary<int, Texture2D>());
            }

            RegisterTexture(m_Texture, (int)TYPE_VIEW.RETRIEVE_MEASURE, (int)mode);

            return m_Texture;
        }


        /// <summary>
        /// Destroy a texture and free the texture.
        /// </summary>
        /// <param name="type">The type of texture</param>
        /// <param name="option">The options</param>
        private void DestroyTexture(int type, int option)
        {

            if (textures.ContainsKey(type) && textures[type].ContainsKey(option))
            {
                textures[type][option] = null;
                textures[type].Remove(option);
                if (textures[type].Count == 0)
                {
                    textures.Remove(type);
                }
            }
        }

        /// <summary>
        /// Destroy all textures
        /// </summary>
        private void DestroyAllTexture()
        {
            if (cameraIsReady)
            {
                foreach (TextureRequested t in texturesRequested)
                {
                    DestroyTexture(t.type, t.option);
                }
                texturesRequested.Clear();
            }
        }

        /// <summary>
        /// Destroy a texture created with CreateTextureViewType
        /// </summary>
        /// <param name="type">The option of the texture</param>
        private void DestroyTextureViewType(int option)
        {
            DestroyTexture((int)TYPE_VIEW.GET_VIEW, option);
        }

        /// <summary>
        /// Destroy a texture created with CreateTextureImageType
        /// </summary>
        /// <param name="type">The option of the texture</param>
        private void DestroyTextureImageType(int option)
        {
            DestroyTexture((int)TYPE_VIEW.RETRIEVE_IMAGE, option);
        }

        /// <summary>
        /// Destroy a texture created with CreateTextureMeasureType
        /// </summary>
        /// <param name="type">The option of the texture</param>
        private void DestroyTextureMeasureType(int option)
        {
            DestroyTexture((int)TYPE_VIEW.RETRIEVE_MEASURE, option);
        }

        /// <summary>
        /// Retrieves a texture previously created
        /// </summary>
        /// <param name="type">The type of texture</param>
        /// <param name="mode">The mode</param>
        /// <returns>The texture</returns>
        public Texture2D GetTexture(TYPE_VIEW type, int mode)
        {
            if (HasTexture((int)type, mode))
            {
                return textures[(int)type][mode];
            }
            return null;
        }

        /// <summary>
        /// Check if a texture is available
        /// </summary>
        /// <param name="type">The type of texture</param>
        /// <param name="mode">The mode</param>
        /// <returns>True if the texture is available</returns>
        private bool HasTexture(int type, int mode)
        {
            if (cameraIsReady)
            {
                return textures.ContainsKey((int)type) && textures[type].ContainsKey((int)mode);
            }
            return false;
        }
        /// <summary>
        /// Width of the images returned by the ZED
        /// </summary>
        public int ImageWidth
        {
            get
            {
                return imageWidth;
            }
        }

        /// <summary>
        /// Returns the height of the image
        /// </summary>
        public int ImageHeight
        {
            get
            {
                return imageHeight;
            }
        }
        /// <summary>
        /// True when the initialization of the ZED has succeeded. 
        /// </summary>
        public bool CameraIsReady
        {
            get
            {
                return cameraIsReady;
            }
        }
        /// <summary>
        /// The projection which corresponds to the traits of the ZED
        /// </summary>
        public Matrix4x4 Projection
        {
            get
            {
                return projection;
            }
        }

        /// <summary>
        /// Sets a filtering value for the disparity map (and by extension the depth map). The function should be called before the grab to be taken into account.
        /// </summary>
        /// <param name="threshold"> a value in [1,100]. A lower value means more confidence and precision (but less density), an upper value reduces the filtering (more density, less certainty). Other value means no filtering.
        ///</param>
        public void SetConfidenceThreshold(int threshold)
        {
            AssertCameraIsReady();
            dllz_set_confidence_threshold(threshold);
        }

        /// <summary>
        /// Set the maximum distance of depth/disparity estimation (all values after this limit will be reported as TOO_FAR value)
        /// </summary>
        /// <param name="distanceMax"> maximum distance in the defined UNIT</param>
        public void SetDepthMaxRangeValue(float distanceMax)
        {
            AssertCameraIsReady();
            dllz_set_depth_max_range_value(distanceMax);
        }

        /// <summary>
        /// Returns the current fps
        /// </summary>
        /// <returns>The current fps</returns>
        public float GetCameraFPS()
        {
            return dllz_get_camera_fps();
        }

        /// <summary>
        /// Get the parameters of the Camera
        /// </summary>
        /// <returns>The current parameters</returns>
        public CameraInformations GetCameraInformation()
        {
            AssertCameraIsReady();
            IntPtr p = dllz_get_camera_information();

            if (p == IntPtr.Zero)
            {
                return new CameraInformations();
            }
            CameraInformations parameters = (CameraInformations)Marshal.PtrToStructure(p, typeof(CameraInformations));

            return parameters;
        }



        /// <summary>
        /// Gets the zed firmware
        /// </summary>
        /// <returns>The firmware</returns>
        public int GetZEDFirmware()
        {
            return dllz_get_zed_firmware();
        }

        /// <summary>
        /// Returns the vertical field of view in radians
        /// </summary>
        /// <returns>The field of view</returns>
        public float GetFOV()
        {
            AssertCameraIsReady();
            return GetCameraInformation().calibParameters.leftCam.vFOV * Mathf.Deg2Rad;
        }

        /// <summary>
        /// Gets the calibration status 
        /// </summary>
        /// <returns>The calibration status</returns>
        public ZED_SELF_CALIBRATION_STATE GetSelfCalibrationStatus()
        {
            return (ZED_SELF_CALIBRATION_STATE)dllz_get_self_calibration_state();
        }


        /// <summary>
        /// Compute textures from the ZED, the new textures will not be displayed until an event is sent to the Render Thread.
        /// </summary>
        public void RetrieveTextures()
        {
            AssertCameraIsReady();
            dllz_update_textures();
        }

        /// <summary>
        /// Get the number of frame dropped since grab() has been called for the first time 
        /// Based on camera time stamp and fps comparison.
        /// </summary>
        /// <returns>number	of frame dropped since first grab() call.</returns>
        public uint GetFrameDroppedCount()
        {
            AssertCameraIsReady();
            return dllz_get_frame_dropped_count();
        }

        /// <summary>
        ///  Gets the position of the camera and the current state of the Tracker
        /// </summary>
        /// <param name="rotation">the quaternion will be filled with the current rotation of the camera depending on the reference</param>
        /// <param name="position">the vector will be filled with the current position of the camera depending on the reference</param>
        /// <param name="referenceType">The reference type will fill the quaternion and vector with either the diffrences between the last pose(CAMERA) or the cumul of poses (WORLD)</param>
        /// <returns>A tracking frame state</returns>
        public TRACKING_FRAME_STATE GetPosition(ref Quaternion rotation, ref Vector3 position, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            AssertCameraIsReady();
            return (TRACKING_FRAME_STATE)dllz_get_position(ref rotation, ref position, (int)referenceType);
        }


        /// <summary>
        /// Get the current position of the camera with an optionnal transformation of the camera frame/motion tracking frame.
        /// 
        /// </summary>
        /// <param name="rotation">the quaternion will be filled with the current rotation of the camera depending on the reference</param>
        /// <param name="position">the vector will be filled with the current position of the camera depending on the reference</param>
        /// <param name="targetQuaternion"></param>
        /// <param name="targetTranslation"></param>
        /// <param name="mat_type"></param>
        /// <returns></returns>
        public TRACKING_FRAME_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, REFERENCE_FRAME mat_type = REFERENCE_FRAME.WORLD)
        {
            AssertCameraIsReady();
            return (TRACKING_FRAME_STATE)dllz_get_position_at_target_frame(ref rotation, ref translation, ref targetQuaternion, ref targetTranslation, (int)mat_type);
        }


        public TRACKING_FRAME_STATE GetPosition(ref Pose pose)
        {
            AssertCameraIsReady();
            return (TRACKING_FRAME_STATE)dllz_get_position_data(ref pose, 0/*WORLD_FRAME*/);
        }

        /// <summary>
        /// Converts a matrix to a float array
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        static public float[] Matrix4x42Float(Matrix4x4 m)
        {
            float[] f = new float[16];
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    f[i * 4 + j] = m[i, j];
                }
            }
            return f;
        }

        /// <summary>
        ///  Converts a matrix to a float array
        /// </summary>
        /// <param name="m"></param>
        /// <param name="f"></param>
        static public void Matrix4x42Float(Matrix4x4 m, float[] f)
        {
            if (f == null) return;
            if (f.Length != 16) return;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    f[i * 4 + j] = m[i, j];
                }
            }

        }

        /// <summary>
        /// Converts a float array to a matrix
        /// </summary>
        /// <param name="m"></param>
        /// <param name="f"></param>
        static public void Float2Matrix(ref Matrix4x4 m, float[] f)
        {
            if (f == null) return;
            if (f.Length != 16) return;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    m[i, j] = f[i * 4 + j];
                }
            }
        }
        /// <summary>
        /// Set settings of the camera
        /// </summary>
        /// <param name="settings">The setting which will be changed</param>
        /// <param name="value">The value</param>
        /// <param name="usedefault">will set default (or automatic) value if set to true (value (int) will not be taken into account)</param>
        public void SetCameraSettings(CAMERA_SETTINGS settings, int value, bool usedefault = false)
        {
            cameraSettingsManager.SetCameraSettings(settings, value, usedefault);
        }

        /// <summary>
        /// Get the value from a setting of the camera
        /// </summary>
        /// <param name="settings"></param>
        public int GetCameraSettings(CAMERA_SETTINGS settings)
        {
            return cameraSettingsManager.GetCameraSettings(settings);
        }

        /// <summary>
        /// Load the camera settings (brightness, contrast, hue, saturation, gain, exposure)
        /// </summary>
        /// <param name="path"></param>
        public void LoadCameraSettings(string path)
        {
            cameraSettingsManager.LoadCameraSettings(instance, path);
        }

        /// <summary>
        /// Save the camera settings (brightness, contrast, hue, saturation, gain, exposure)
        /// </summary>
        /// <param name="path"></param>
        public void SaveCameraSettings(string path)
        {
            cameraSettingsManager.SaveCameraSettings(path);
        }

        /// <summary>
        /// Retrieves camera settings from the camera
        /// </summary>
        public void RetrieveCameraSettings()
        {
            cameraSettingsManager.RetrieveSettingsCamera(instance);
        }

        /// <summary>
        /// Returns a copy of the camera settings, cannot be modified
        /// </summary>
        /// <returns></returns>
        public ZEDCameraSettingsManager.CameraSettings GetCameraSettings()
        {
            return cameraSettingsManager.Settings;
        }

        /// <summary>
        /// Set all the settings registered, to the camera
        /// </summary>
        public void SetCameraSettings()
        {
            cameraSettingsManager.SetSettings(instance);
        }


        /// <summary>
        /// The function checks if ZED cameras are connected, can be called before instantiating a Camera object
        /// </summary>
        /// <remarks> On Windows, only one ZED is accessible so this function will return 1 even if multiple ZED are connected.</remarks>
        /// <returns>the number of connected ZED</returns>
        public static bool IsZedConnected()
        {
            return Convert.ToBoolean(dllz_is_zed_connected());
        }

        /// <summary>
        /// The function return the version of the currently installed ZED SDK
        /// </summary>
        /// <returns>ZED SDK version as a string with the following format : MAJOR.MINOR.PATCH</returns>
        public static string GetSDKVersion()
        {
            return PtrToStringUtf8(dllz_get_sdk_version());
        }

        private void AssertCameraIsReady()
        {

            if (!cameraIsReady || !pluginIsReady)
                throw new Exception("Camera is not connected, init was not called or a dependency problem occurred");
        }

        /// <summary>
        /// retrieve the images from the ZED and send an event to update the textures
        /// </summary>
        public void UpdateTextures()
        {
            //Retrieves the images from the ZED
            RetrieveTextures();
            GL.IssuePluginEvent(GetRenderEventFunc(), 1);
        }

        /// <summary>
        /// Get the current depth value in the UNIT
        /// </summary>
        /// <param name="x">The position in the image in width</param>
        /// <param name="y">The position in the image in height</param>
        /// <returns></returns>
        public float GetDepthValue(uint x, uint y)
        {
            AssertCameraIsReady();
            float posX = ImageWidth * (float)x / (float)Screen.width;
            float posY = ImageHeight * (1 - (float)y / (float)Screen.height);
            return dllz_get_depth_value((uint)posX, (uint)posY);
        }

        /// <summary>
        /// Get the current depth value in the UNIT
        /// </summary>
        /// <param name="position">The position with a vector, the component Z is unused, this position has to be in the screen dimensions "GetDepthValue(Input.mousePosition)" works only if the zed image fills the whole screen</param>
        /// <returns></returns>
        public float GetDepthValue(Vector3 position)
        {
            AssertCameraIsReady();
            float posX = ImageWidth * (float)position.x / (float)Screen.width;
            float posY = ImageHeight * (1 - (float)position.y / (float)Screen.height);
            return dllz_get_depth_value((uint)posX, (uint)posY);
        }

    }
} // namespace sl