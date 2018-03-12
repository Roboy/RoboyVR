using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;
using ROSBridgeLib.sensor_msgs;
using ROSBridgeLib;
using ROSBridgeLib.custom_msgs;

/// <summary>
/// BeRoboymanager has different tasks to do:
///
///     -# Keep track of user movement and translate roboy when in specific view modes
///     -# Convert received images into textures which can then be rendered on screen
///     -# Send tracking messages over the rosbridge to gazebo/ real roboy
/// </summary>
public class BeRoboyManager : Singleton<BeRoboyManager>
{


    #region PUBLIC_MEMBER_VARIABLES

    /// <summary>
    /// Set whether head movement should be tracked or not.
    /// </summary>
    public bool TrackingEnabled = false;

    /// <summary>
    /// Reference to the render texture in which the Zed feed gets pushed into.
    /// </summary>
    public RenderTexture RT_Zed;

    /// <summary>
    /// Reference to the render texture in which the Simulation feed gets pushed into.
    /// </summary>
    public RenderTexture RT_Simulation;

    #endregion PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// The HMD main camera.
    /// </summary>
    [SerializeField]
    private GameObject m_Cam;

    /// <summary>
    /// Texture in which the received simulation images get drawn.
    /// </summary>
    private Texture2D m_TexSim;

    /// <summary>
    /// Texture in which the received zed images get drawn.
    /// </summary>
    private Texture2D m_TexZed;

    /// <summary>
    /// Is the main camera initialized or not.
    /// </summary>
    private bool m_CamInitialized = false;

    /// <summary>
    /// Variable to determine if headset was rotated.
    /// </summary>
    private float m_CurrentAngleX = 0.0f;

    /// <summary>
    /// Variable to determine if headset was rotated.
    /// </summary>
    private float m_CurrentAngleY = 0.0f;

    /// <summary>
    /// Color array for the simulation image conversion.
    /// </summary>
    private Color[] m_ColorArraySim = new Color[640 * 480];

    /// <summary>
    /// Color array for the zed image conversion.
    /// </summary>
    private Color[] m_ColorArrayZed = new Color[1280 * 720];

    /// <summary>
    /// Reference to the tools controller (right).
    /// </summary>
    private SteamVR_TrackedObject m_Controller;

    #endregion PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initialize textures.
    /// </summary>
    void Awake()
    {
        m_TexSim = new Texture2D(640, 480);
        m_TexZed = new Texture2D(1280, 720);
    }


    void Start()
    {

        //Looking for the HMD camera in scene
        if (!m_CamInitialized)
        {
            tryInitializeCamera();
        }
        else
        {
            Debug.Log("No Camera found!");
        }

        m_Controller = ControllerManager.Instance.ControllerForTools;


    }

    void Update()
    {

        // Looking for the HMD camera in scene.
        if (!m_CamInitialized)
        {
            tryInitializeCamera();
        }
        // If the camera is found, move and rotate Roboy accordingly.
        else
        {
            if (TrackingEnabled)
                translateRoboy();
        }

    }
    #endregion //MONOBEHAVIOR_METHODS


    #region PUBLIC_METHODS

    /// <summary>
    /// Primary function to receive image (zed) messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveZedMessage(ImageMsg image)
    {
        RefreshZedImage(image);
    }

    /// <summary>
    /// Primary function to receive image (simulation) messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveSimMessage(ImageMsg image)
    {
        RefreshSimImage(image);
    }

    /// <summary>
    /// Function to publish ExternalJoint messages via ROS.
    /// </summary>
    /// <param name="jointNames"></param>
    /// <param name="angles"></param>
    public void PublishExternalJoint(List<string> jointNames, List<float> angles)
    {
        ExternalJointMsg msg = new ExternalJointMsg(jointNames, angles);

        ROSBridge.Instance.Publish(RoboyHeadPublisher.GetMessageTopic(), msg);
    }

    /// <summary>
    /// Function to publish Position messages via ROS.
    /// </summary>
    /// <param name="pos"></param>
    public void PublishRoboyPosition(Vector3 pos)
    {
        RoboyPositionMsg msg = new RoboyPositionMsg(pos.x, pos.y, pos.z);
        Debug.Log("MESSAGE: ROBOY POS: " + msg.ToYAMLString());
        ROSBridge.Instance.Publish(RoboyPositionPublisher.GetMessageTopic(), msg);
    }

    #endregion PUBLIC_METHODS


    #region PRIVATE_METHODS

    /// <summary>
    /// Renders the received images from the zed camera
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void RefreshZedImage(ImageMsg image)
    {
        //Get the image as an array from the message.
        byte[] image_temp = image.GetImage();

        int j = 0;
        for (int i = 0; i < image_temp.Length; i += 3)
        {
            m_ColorArrayZed[j].b = image_temp[i] / (float)255;
            m_ColorArrayZed[j].g = image_temp[i + 1] / (float)255;
            m_ColorArrayZed[j].r = image_temp[i + 2] / (float)255;

            m_ColorArrayZed[j].a = 1f;
            j++;
        }

        // Load data into the texture.
        m_TexZed.SetPixels(m_ColorArrayZed);
        m_TexZed.Apply();

        Graphics.Blit(m_TexZed, RT_Zed);
    }

    /// <summary>
    /// Renders the received images from the simulation.
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void RefreshSimImage(ImageMsg image)
    {

        // Get the image as an array from the message.
        byte[] image_temp = image.GetImage();


        int j = 0;
        for (int i = 0; i < image_temp.Length; i += 3)
        {
            m_ColorArraySim[j].r = image_temp[i] / (float)255;
            m_ColorArraySim[j].g = image_temp[i + 1] / (float)255;
            m_ColorArraySim[j].b = image_temp[i + 2] / (float)255;

            m_ColorArraySim[j].a = 1f;
            j++;
        }

        // Load data into the texture.
        m_TexSim.SetPixels(m_ColorArraySim);
        m_TexSim.Apply();

        Graphics.Blit(m_TexSim, RT_Simulation);
    }

    /// <summary>
    /// Turn Roboy with the movement of the HMD.
    /// </summary>
    private void translateRoboy()
    {
        // References to roboy parts we need for rotation/ translation
        Transform head_parent = transform.GetChild(0).Find("head");
        Transform head_pivot = head_parent.GetChild(0);
        Transform torso_pivot = transform.GetChild(0).Find("torso_pivot");


        // Check whether the user has rotated the headset in x direction or not
        if (m_CurrentAngleX != m_Cam.transform.eulerAngles.x)
        {
            // If the headset was rotated, rotate roboy
            head_parent.RotateAround(head_pivot.position, Vector3.right, m_Cam.transform.eulerAngles.x - m_CurrentAngleX);
        }
        m_CurrentAngleX = m_Cam.transform.eulerAngles.x;

        // Check whether the user has rotated the headset in y direction or not
        if (m_CurrentAngleY != m_Cam.transform.eulerAngles.y)
        {
            // If the headset was rotated, rotate roboy
            head_parent.RotateAround(head_pivot.position, Vector3.up, m_Cam.transform.eulerAngles.y - m_CurrentAngleY);
        }
        m_CurrentAngleY = m_Cam.transform.eulerAngles.y;

        // Move roboy accordingly to headset movement
        Quaternion headRotation = UnityEngine.XR.InputTracking.GetLocalRotation(UnityEngine.XR.XRNode.Head);
        transform.position = m_Cam.transform.position + (headRotation * Vector3.forward) * (-0.3f);

        // Publish position to gazebo
        PublishRoboyPosition(transform.position);

        // The torso of roboy will be rotated towards the (right)controller position
        torso_pivot.transform.LookAt(new Vector3(m_Controller.transform.position.x, torso_pivot.transform.position.y, m_Controller.transform.position.z));

        // Send rotation data via ROS
        // Convert the headset rotation from unity coordinate spaze to gazebo coordinates
        Quaternion rot = GazeboUtility.UnityRotationToGazebo(headRotation);
        float x_angle = 0.0f;
        float y_angle = 0.0f;
        // Angle for torso rotation
        float t_angle = 0.0f;

        // Convert head rotation
        if (rot.eulerAngles.x > 180)
        {
            y_angle = (rot.eulerAngles.x - 360) * Mathf.Deg2Rad;
        }
        else
        {
            y_angle = rot.eulerAngles.x * Mathf.Deg2Rad;
        }

        x_angle = rot.eulerAngles.z * Mathf.Deg2Rad;

        // Now convert torso rotation
        if (torso_pivot.eulerAngles.y > 180)
        {
            t_angle = (torso_pivot.eulerAngles.y - 360) * Mathf.Deg2Rad;
        }
        else
        {
            t_angle = torso_pivot.eulerAngles.y * Mathf.Deg2Rad;
        }

        // Determine which joints should me modified
        List<string> joints = new List<string>();
        // X rotation
        joints.Add("neck_3");
        // Y rotation
        joints.Add("neck_4");
        // Body rotation
        joints.Add("spine_1");

        // Determine the angle for the joints
        List<float> angles = new List<float>();
        // Add the x-angle of the headset after conversion from unity to ros
        angles.Add(x_angle);
        // Add the y-angle of the headset after conversion from unity to ros
        angles.Add(y_angle);
        // Add the torso rotation angle after conversion from unity to ros
        angles.Add(t_angle * (-1.0f));
        // Start sending the actual message
        PublishExternalJoint(joints, angles);
    }

    /// <summary>
    /// Looking for the main camera in the scene, which can be attached to Roboy.
    /// </summary>
    private void tryInitializeCamera()
    {
        // Look for a camera and initialize it.
        m_Cam = GameObject.FindGameObjectWithTag("MainCamera");
        if (m_Cam != null)
            m_CamInitialized = true;
    }

    #endregion PRIVATE_METHODS
}