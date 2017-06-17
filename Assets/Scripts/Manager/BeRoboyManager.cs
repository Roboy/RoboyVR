using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;
using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using UnityEngine.UI;
using System.IO;


/// <summary>
/// BeRoboymanager has different tasks to do:
///
///     -# Keep track of user movement and translate roboy when in specific view modes
///     -# Convert received images into textures which can then be rendered on screen
///     -# FUTURE: Send tracking messages over the rosbridge to gazebo/ real roboy
/// </summary>
public class BeRoboyManager : Singleton<BeRoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES

    /// <summary>
    /// Set whether head movement should be tracked or not.
    /// </summary>
    public bool TrackingEnabled = false;

    public RenderTexture RenderTest;

    public RenderTexture RT_Zed;
    public RenderTexture RT_Simulation;

    #endregion PUBLIC_MEMBER_VARIABLES

    

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// The HMD main camera.
    /// </summary>
    [SerializeField]
    private GameObject m_Cam;


    /// <summary>
    /// Texture in which the received images get drawn.
    /// </summary>
    private Texture2D m_Tex;

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
    private float m_current_Angle = 0.0f;


    /// <summary>
    /// Resolution determines array size e.g. 1280*720
    /// </summary>
    private Color[] colorArray = new Color[1280*720];

    /// <summary>
    /// Color array for the simulation image conversion.
    /// </summary>
    private Color[] m_colorArraySim = new Color[640 * 480];

    /// <summary>
    /// Color array for the zed image conversion.
    /// </summary>
    private Color[] m_colorArrayZed = new Color[1280 * 720];

    #endregion PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    /// <summary>
    /// Initialize texture
    /// </summary>
    void Awake()
    {
        //Initialize the textures
        m_Tex = new Texture2D(640, 480);
        m_TexSim = new Texture2D(640, 480);
        m_TexZed = new Texture2D(1280, 720);
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
	
	void Update () {

        // Looking for the HMD camera in scene.
        if (!m_CamInitialized)
        {
            TryInitializeCamera();
        }
        // If the camera is found, move and rotate Roboy accordingly.
        else
        {
            if(TrackingEnabled)
            TranslateRoboy();
        }
    }
    #endregion //MONOBEHAVIOR_METHODS


    #region PUBLIC_METHODS
    /// <summary>
    /// Primary function to receive messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveMessage(ImageMsg image)
    {
        RefreshImage(image);
    }


    /// <summary>
    /// Primary function to receive messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveZedMessage(ImageMsg image)
    {
        RefreshZedImage(image);
    }

    /// <summary>
    /// Primary function to receive messages from ROSBridge. Renders the received images.
    /// </summary>
    /// <param name="msg">JSON msg containing roboy pose.</param>
    public void ReceiveSimMessage(ImageMsg image)
    {
        RefreshSimImage(image);
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
            m_colorArrayZed[j].b = image_temp[i] / (float)255;
            m_colorArrayZed[j].g = image_temp[i + 1] / (float)255;
            m_colorArrayZed[j].r = image_temp[i + 2] / (float)255;

            m_colorArrayZed[j].a = 1f;
            j++;
        }

        // Load data into the texture.
        m_TexZed.SetPixels(m_colorArrayZed);
        m_TexZed.Apply();

        Graphics.Blit(m_TexZed, RT_Zed);
    }

    /// <summary>
    /// Renders the received images from the simulation
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void RefreshSimImage(ImageMsg image)
    {
        //Get the image as an array from the message.
        byte[] image_temp = image.GetImage();

        int j = 0;
        for (int i = 0; i < image_temp.Length; i += 3)
        {
            m_colorArraySim[j].b = image_temp[i] / (float)255;
            m_colorArraySim[j].g = image_temp[i + 1] / (float)255;
            m_colorArraySim[j].r = image_temp[i + 2] / (float)255;

            m_colorArraySim[j].a = 1f;
            j++;
        }

        // Load data into the texture.
        m_TexSim.SetPixels(m_colorArrayZed);
        m_TexSim.Apply();

        Graphics.Blit(m_TexSim, RT_Simulation);
    }

    /// <summary>
    /// Renders the received images from the simulation e.g.
    /// </summary>
    /// <param name="msg">JSON msg containing the roboy pose.</param>
    private void RefreshImage(ImageMsg image)
    {
        //Get the image as an array from the message.
        byte[] image_temp = image.GetImage();
        Debug.Log(image_temp.Length);

        //Color[] colorArray = new Color[image_temp.Length / 3];
        int j = 0;
        for (int i = 0; i < image_temp.Length; i += 3)
        {
            colorArray[j].b = image_temp[i] / (float)255;
            colorArray[j].g = image_temp[i + 1] / (float)255;
            colorArray[j].r = image_temp[i + 2] / (float)255;

            colorArray[j].a = 1f;
            j++;
        }

        // Load data into the texture.
        m_Tex.SetPixels(colorArray);
        m_Tex.Apply();

        Graphics.Blit(m_Tex, RenderTest);
        // Store the texture in temporary png.
        //saveTextureToFile(m_Tex);
        //// Load the texture from a temporary png.
        //Texture2D t = loadTextureFromFile("temp.png");
        //Rect rec = new Rect(0, 0, t.width, t.height);
        //Sprite spriteToUse = Sprite.Create(t, rec, new Vector2(0.5f, 0.5f), 100);
        ////Finding the image to be replaced by the simulation feed.
        //m_SimulatedImg.sprite = spriteToUse;

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
        var filestream = File.Open(Application.dataPath + "/" + filename, FileMode.OpenOrCreate, FileAccess.ReadWrite);
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

