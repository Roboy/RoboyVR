using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;
using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using UnityEngine.UI;
using System.IO;



public class BeRoboyManager : Singleton<BeRoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// The IP address of the machine where the simulation is running.
    /// </summary>
    public string IP = "";

    /// <summary>
    /// Set whether head movement should be tracked or not.
    /// </summary>
    public bool TrackingEnabled = false;

    public Image SimulationCameraFeed;
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

    #region MONOBEHAVIOR_METHODS

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
        m_Tex = new Texture2D(640, 480);

        

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
            if(TrackingEnabled)
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
        if (m_ROSInitialized)
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
        byte[] image_temp = flipBytes(image.GetImage(), 640);
        //Debug.Log(image_temp);
        Color[] colorArray = new Color[image_temp.Length/3];
        for (int i = 0; i < image_temp.Length; i+=3)
        {
            Color color = new Color(image_temp[i] / (float)255, image_temp[i + 1] / (float)255,
                image_temp[i + 2] / (float)255, 1f);
            colorArray[i / 3] = color;
        }
        System.Array.Reverse(colorArray);

        // Load data into the texture.
        m_Tex.SetPixels(colorArray);
	// Store the texture in temporary png.
        saveTextureToFile(m_Tex);
        // Load the texture from a temporary png.
	Texture2D t = loadTextureFromFile("temp.png");
	Rect rec = new Rect(0, 0, t.width, t.height);
        Sprite spriteToUse = Sprite.Create(t, rec, new Vector2(0.5f, 0.5f), 100);
        //Finding the image to be replaced by the simulation feed.
        GameObject Img = GameObject.FindGameObjectWithTag("SimImg");
        Img.GetComponent<Image>().sprite = spriteToUse;
	
        //Should replace the images by setting it via overrideSprite using the Texture
        //m_Pan.GetComponent<Image>().sprite = Sprite.Create(m_Tex, new Rect(0.0f, 0.0f, m_Tex.width, m_Tex.height), new Vector2(0.5f, 0.5f), 0.10f);
        
    }

    /// <summary>
    /// TEST FUNCTION TO SAVE TEXTURE TO ASSETS FOLDER
    /// </summary>
    /// <param name="tex"></param>
    private void saveTextureToFile(Texture2D tex)
    {
        string filename = "temp.png";
        byte[] bytes = tex.EncodeToPNG();
        var filestream = File.Open(Application.dataPath + "/" + filename, FileMode.Create);
        var binarywriter = new BinaryWriter(filestream);
        binarywriter.Write(bytes);
        filestream.Close();
    }
	
    /// <summary>
    /// TEST FUNCTION TO LOAD TEXTURE FROM ASSETS FOLDER
    /// </summary>
    /// <param name="filename"></param>
    private Texture2D loadTextureFromFile(string filename)
    {
        Texture2D tex = null;
        byte[] fileData;
        string filePath = Application.dataPath + "/" + filename;

        fileData = File.ReadAllBytes(filePath);
        tex = new Texture2D(2, 2);
        tex.LoadImage(fileData); //..this will auto-resize the texture dimensions.
        return tex;
    }
	

    /// <summary>
    /// DOES NOT WORK KINDA
    /// </summary>
    /// <param name="bytes"></param>
    /// <param name="width"></param>
    /// <returns></returns>
    private byte[] flipBytes(byte[] bytes, int width)
    {
        BitArray bits = new BitArray(bytes);
        BitArray flippedBits = new BitArray(bits);

        for (int i = 0; i < bits.Length; i += 640)
        {
            for (int j = 0, k = width - 1; j < width; ++j, --k)
            {
                flippedBits[i + j] = bits[i + k];
            }
        }
        byte[] flippedBytes = new byte[bytes.Length];
        flippedBits.CopyTo(flippedBytes, 0);
        return flippedBytes;
    }

    //private void OnGUI()
    //{
    //    GUI.DrawTexture(new Rect(10, 10, 300, 300), m_Tex, ScaleMode.ScaleToFit, true, 10.0F);

    //}

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

