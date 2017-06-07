using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;
using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using UnityEngine.UI;



public class BeRoboyManager : Singleton<BeRoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// The IP address of the machine where the simulation is running.
    /// </summary>
    public string IP = "";
    #endregion PUBLIC_MEMBER_VARIABLES


    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// ROSBridge websocket.
    /// </summary>
    private ROSBridgeWebSocketConnection m_Ros = null;

    /// <summary>
    /// Image message, from simulation e.g. 
    /// </summary>
    private ImageMsg m_ImageMessage;

    /// <summary>
    /// Variable to check if the ROS connection is working!
    /// </summary>
    private bool m_ROSInitialized = false;

    /// <summary>
    /// The HMD main camera.
    /// </summary>
    [SerializeField]
    private GameObject m_Cam;

    /// <summary>
    /// The panel @which the video stream should be displayed.
    /// </summary>
    [SerializeField]
    private CameraPanel m_Pan;

    /// <summary>
    /// Texture in which the received images get drawn.
    /// </summary>
    private Texture2D m_Tex;

    /// <summary>
    /// Is the main camera initialized or not.
    /// </summary>
    private bool m_CamInitialized = false;

    /// <summary>
    /// Variable to determine if headset was rotated.
    /// </summary>
    private float m_current_Angle = 0.0f;

    #endregion PRIVATE_MEMBER_VARIABLES

    #region //MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initialize ROSBridge.
    /// </summary>
    void Awake()
    {
        if (string.IsNullOrEmpty(IP))
            return;

        m_Ros = new ROSBridgeWebSocketConnection("ws://" + IP, 9090);

        // DOES NOT WORK! m_Ros is never null if you call the Constructor, WAIT UNTILL SIMON IMPLEMENTS UDP BROADCAST WITH ROS CONFIGURATION, GET THE IP ADDRESS FROM THE BROADCAST.
        if (m_Ros != null)
        {
            m_Ros.AddSubscriber(typeof(RoboyCameraSubscriber));
            //m_Ros.AddServiceResponse(typeof(RoboyServiceResponse));
            //m_Ros.AddPublisher(typeof(RoboyPosePublisher));
            m_Ros.Connect();
            m_ROSInitialized = true;

            Debug.Log("ROS successfully initialized!");
        }
        else
        {
            Debug.LogWarning("ROS could not be initialized!");
        }

        //Initialize the texture
        m_Tex = new Texture2D(2, 2);

    }


    void Start () {

        //Looking for the HMD camera in scene
        if (!m_CamInitialized)
        {
            TryInitializeCamera();
        }
        else
        {
            Debug.Log("No Camera found!");
        }

    }
	
	// Update is called once per frame
	void Update () {

        //Looking for the HMD camera in scene.
        if (!m_CamInitialized)
        {
            TryInitializeCamera();
        }
        //If the camera is found, move and rotate Roboy accordingly.
        else
        {
            TranslateRoboy();
        }

        //Get new (Image)messages.
        if (m_ROSInitialized)
        {
            m_Ros.Render();
        }

    }

    /// <summary>
    /// Disconnect from the simulation when Unity is not running.
    /// </summary>
    void OnApplicationQuit()
    {
        if (!m_ROSInitialized)
            m_Ros.Disconnect();
    }
    #endregion //MONOBEHAVIOR_METHODS


    #region PUBLIC_METHODS
    /// <summary>
    /// Primary function to receive messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveMessage(ImageMsg image)
    {
        //Debug.Log("Received message");
        RefreshImage(image);
    }
    #endregion PUBLIC_METHODS


    #region PRIVATE_METHODS

    /// <summary>
    /// Renders the received images from the simulation e.g.
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void RefreshImage(ImageMsg image)
    {
        //Get the image as an array from the message.
        byte[] image_temp = image.GetImage();

        // Load data into the texture.
        m_Tex.LoadImage(image_temp);

        // Assign texture to renderer's material.
        GetComponent<Renderer>().material.mainTexture = m_Tex;

        //Should replace the images by setting it via overrideSprite using the Texture
        m_Pan.GetComponent<Image>().overrideSprite = Sprite.Create(m_Tex, new Rect(0.0f, 0.0f, m_Tex.width, m_Tex.height), new Vector2(0.5f, 0.5f), 0.0f);

    }

    /// <summary>
    /// Turn Roboy with the movement of the HMD.
    /// </summary>
    private void TranslateRoboy()
    {
        //Check whether the user has rotated the headset or not
        if (m_current_Angle != m_Cam.transform.eulerAngles.y)
        {
            //If the headset was rotated, rotate roboy
            transform.RotateAround(m_Cam.transform.localPosition, Vector3.up, m_Cam.transform.eulerAngles.y - m_current_Angle);
        }
        m_current_Angle = m_Cam.transform.eulerAngles.y;

        //Move roboy accordingly to headset movement
        Quaternion headRotation = InputTracking.GetLocalRotation(VRNode.Head);
        transform.position = m_Cam.transform.position + (headRotation * Vector3.forward) * (-0.3f);
    }

    /// <summary>
    /// Looking for the main camera in the scene, which can be attached to Roboy.
    /// </summary>
    private void TryInitializeCamera()
    {
        //Look for a camera and initialize it.
        m_Cam = GameObject.FindGameObjectWithTag("MainCamera");
        if (m_Cam != null)
            m_CamInitialized = true;
    }

    #endregion PRIVATE_METHODS
}

