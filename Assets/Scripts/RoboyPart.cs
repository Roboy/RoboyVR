using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RoboyPart : MonoBehaviour
{

    #region PUBLIC_VARIABLES
    [HideInInspector]
    public Vector3 Position;
    [HideInInspector]
    public Quaternion Rotation;

    [HideInInspector]
    public Dictionary<ModeManager.Panelmode, Category> Categories =
        new Dictionary<ModeManager.Panelmode, Category>();

    /// <summary>
    /// number of motors of that part of roboy
    /// </summary>
    [HideInInspector]
    public int MotorCount;
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Prebab of a rawimage (with collider if desired) used as an icon prefab that displays notification sign next to Roboy
    /// Issue: since the roboy model is already created, it is easier to manually create instances based on prefabs 
    /// (and look for this instance)
    /// </summary>
    static private GameObject m_IconPrefab;

    /// <summary>
    /// reference to parent object containing all bones / body parts, needed for position calculation
    /// </summary>
    static private GameObject m_Roboy;


    /// <summary>
    /// List containing all received error notifications
    /// </summary>
    private List<Notification> m_myErrors = new List<Notification>();

    /// <summary>
    /// List containing all received warnings
    /// </summary>
    private List<Notification> m_myWarnings = new List<Notification>();

    /// <summary>
    /// List containing all received debugs
    /// </summary>
    private List<Notification> m_myDebugs = new List<Notification>();

    /// <summary>
    /// List containing all received infos or similar additional information
    /// </summary>
    private List<Notification> m_myInfos = new List<Notification>();

    /// <summary>
    /// instance of Icon for respective body part
    /// </summary>
    private GameObject m_myInstance;

    /// <summary>
    /// Texture of my icon prefab. since getcomponent rather costly this reference is created. 
    /// DO NOT USE THIS VARIABLE DIRECTLY -> call GetIconImage()
    /// </summary>
    private RawImage m_IconImage;
    #endregion
    #region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// Creates and initializes notification related components
    /// </summary>
    void Awake()
    {
        //notifications related initialisations
        m_myInstance = CreateIconInstance();
        //Important: first find right position, THEN set child/parent relationship
        m_myInstance.transform.position = FindIconPosition();
        m_myInstance.transform.SetParent(transform);
        UpdateNotificationsDisplay();
    }
    #endregion  

    #region PUBLIC_METHODS
    /// <summary>
    /// initialises body part with specified number of motors.
    /// </summary>
    /// <param name="count">count of motors to be initialised</param>
    public void Initialize(int count)
    {
        var enumList = Enum.GetValues(typeof(ModeManager.Panelmode));

        foreach (var enumElem in enumList)
        {
            Categories.Add((ModeManager.Panelmode)enumElem, new Category((ModeManager.Panelmode)enumElem, count));
        }
        MotorCount = count;
    }

    /// <summary>
    /// If errors for this body part registered, this bodypart will display a halo in red
    /// </summary>
    public void DisplayErrors()
    {
        if (m_myErrors.Count == 0) return;
        Component comp = gameObject.transform.Find("Warning"); //warning less important than error -> replace with error
        if (comp != null)
        {
            Destroy(comp.gameObject);
        }
        comp = transform.Find("Error");
        if (comp == null)
        {
            GameObject obj = Instantiate(Resources.Load("UI/Error")) as GameObject;
            obj.name = "Error";
            obj.transform.parent = transform;
            obj.transform.localPosition = Vector3.zero;
        }
    }

    /// <summary>
    /// Disables any red halo, does not disable the halo if it's a warning
    /// </summary>
    public void DisableErrors()
    {
        Component comp = gameObject.transform.Find("Error");
        if (comp != null)
        {
            Destroy(comp.gameObject);
        }
    }

    /// <summary>
    /// If warnings for this body part registered, this bodypart will display a halo in orange.
    /// If an error is displayed, nothing changes. 
    /// </summary>
    /// 
    public void DisplayWarnings()
    {
        if (m_myWarnings.Count == 0)
            return;
        //only if no error or warning displayed
        Component comp = gameObject.transform.Find("Error");
        if (comp && m_myErrors.Count == 0)
        {
            Destroy(comp.gameObject);
        }
        if (!gameObject.transform.Find("Error") && !gameObject.transform.Find("Warning"))
        {
            GameObject obj = Instantiate(Resources.Load("UI/Warning")) as GameObject;
            obj.name = "Warning";
            obj.transform.parent = transform;
            obj.transform.localPosition = Vector3.zero;
        }
    }

    /// <summary>
    /// Disables halo, if halo is for warning notifications. Nothing happens otherwise
    /// </summary>
    public void DisableWarnings()
    {
        Component comp = gameObject.transform.Find("Warning");
        if (comp != null)
        {
            Destroy(comp.gameObject);
        }
    }

    /// <summary>
    /// Updates the icon depending on the currently registered notifications
    /// </summary>
    public void UpdateIcon()
    {
        if (m_myErrors.Count > 0)
        { //if error exist, display error sign
            if (!m_myInstance.activeInHierarchy)
                m_myInstance.SetActive(true);
            Texture errortex = VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.ERROR);
            if (GetIconImage().texture != errortex)
                GetIconImage().texture = errortex;
        }
        else if (m_myWarnings.Count > 0)
        { //if no errors but warnings, display warning icon
            if (!m_myInstance.activeInHierarchy)
                m_myInstance.SetActive(true);
            Texture warningtex = VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.WARNING);
            if (GetIconImage().texture != warningtex)
                GetIconImage().texture = warningtex;
        }
        else
        {//disable otherwise
            m_myInstance.SetActive(false);
        }
    }

    /// <summary>
    /// This updates the halo to display errors or warnings or no halo
    /// </summary>
    public void UpdateNotificationsDisplay()
    {
        UpdateHalos();
        UpdateIcon();
    }

    /// <summary>
    /// Updates the displayed Halo depending on the currently registered notifications
    /// </summary>
    public void UpdateHalos()
    {
        if (m_myErrors.Count > 0 || m_myWarnings.Count > 0)
        {
            DisplayErrors();
            DisplayWarnings();
        }
        else
        {
            DisableErrors();
            DisableWarnings();
        }
    }

    /// <summary>
    /// adds notification to this bodypart. Useful for link between selected body part and respective notifications
    /// </summary>
    /// <param name="note"></param>m
    public void AddNotification(Notification note)
    {
        if (note == null) return;
        switch (note.GetNotificationType())
        {
            case DummyStates.MessageType.DEBUG:
                m_myDebugs.Add(note);
                break;
            case DummyStates.MessageType.WARNING:
                m_myWarnings.Add(note);
                break;
            case DummyStates.MessageType.ERROR:
                m_myErrors.Add(note);
                break;
            case DummyStates.MessageType.INFO:
                m_myInfos.Add(note);
                break;
            default:
                break;
        }
    }

    /// <summary>
    /// Remove notification from internal list
    /// </summary>
    /// <param name="notification">notification to be removed</param>
    public void RemoveNotification(Notification notification)
    {
        switch (notification.GetNotificationType())
        {
            case DummyStates.MessageType.DEBUG:
                m_myDebugs.Remove(notification);
                break;
            case DummyStates.MessageType.WARNING:
                m_myWarnings.Remove(notification);
                break;
            case DummyStates.MessageType.ERROR:
                m_myErrors.Remove(notification);
                break;
            case DummyStates.MessageType.INFO:
                m_myInfos.Remove(notification);
                break;
            default:
                break;
        }
    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// returns RawImage component of the icon instance of this body part
    /// </summary>
    /// <returns></returns>
    private RawImage GetIconImage()
    {
        if (!m_IconImage)
            m_IconImage = m_myInstance.GetComponentInChildren<RawImage>();
        return m_IconImage;
    }

    /// <summary>
    /// creates Icon instance based on prefab
    /// </summary>
    /// <returns></returns>
    private GameObject CreateIconInstance()
    {
        if (!m_IconPrefab)
        {
            m_IconPrefab = Resources.Load("UI/IconPrefab") as GameObject;
            if (!m_IconPrefab)
                Debug.LogError("[RoboyPart] Missing Prefab! Cannot create icons for warnings");
        }
        GameObject tmp = Instantiate(m_IconPrefab);
        tmp.GetComponentInChildren<Canvas>().worldCamera = VRUILogic.Instance.GetCamera();
        tmp.name = "Icon";
        return tmp;
    }


    /// <summary>
    /// finds optimal position of icon in world space
    /// </summary>
    /// <returns></returns>
    private Vector3 FindIconPosition()
    {
        //world space component pos
        Vector3 componentpos, roboyPos, iconPos, direction, imageScale;
        float boundingOffset, imageOffset; // different offsets (distance) to move the image to
        float x, y; //variables to calculate offsets
        float iconWidth = GetIconImage().rectTransform.rect.width;
        float iconHeight = GetIconImage().rectTransform.rect.height;
        Collider collider = GetComponent<Collider>();
        //transform.position returns 
        componentpos = transform.position;
        //worldspace roboy pos
        roboyPos = GetRoboy().transform.position;

        //moving object in respective direction
        direction = (componentpos - roboyPos).normalized;
        direction.z = 0; // we don't want icons in front of Roboy -> set axis = 0
        //move point somewhere outside of bounding box
        boundingOffset = collider.bounds.extents.x * 2 + collider.bounds.extents.y * 2;
        iconPos = componentpos + boundingOffset * direction;
        //since possibly too far, look for closest point on AABB
        iconPos = collider.bounds.ClosestPoint(iconPos);
        //now, add offset of image -> imageOffset == delta distance
        //TODO: convert from pixels to world space....
        imageScale = GetWorldScale(m_IconImage.transform);
        x = iconWidth * imageScale.x / 2;
        y = imageScale.y * iconHeight / 2;
        imageOffset = Mathf.Sqrt(x * x + y * y);
        //imageOffset = 0;
        iconPos += direction * imageOffset;
        return iconPos;//iconPos ;
    }

    /// <summary>
    /// returns absolute world scale of the object
    /// </summary>
    /// <param name="transform">transform for which to find out the overall scale </param>
    /// <returns></returns>
    public Vector3 GetWorldScale(Transform transform)
    {
        Vector3 worldScale = transform.localScale;
        Transform current = transform.parent;
        //apply each parent's scale
        while (current)
        {
            worldScale = Vector3.Scale(worldScale, current.localScale);
            current = current.parent;
        }

        return worldScale;
    }

    /// <summary>
    /// returns GameObject with whole roboy
    /// </summary>
    /// <returns></returns>
    private GameObject GetRoboy()
    {
        if (!m_Roboy)
        {
            Transform current = transform;
            while (current.transform.parent)
            {
                current = current.parent;
            }
            m_Roboy = current.gameObject;
        }
        return m_Roboy;
    }
    #endregion
}


public class Category
{
    public ModeManager.Panelmode Mode;

    public Dictionary<string, Motor> Motors = new Dictionary<string, Motor>();

    public Category(ModeManager.Panelmode mode, int count)
    {
        Mode = mode;
        initializeMotors(count);
    }

    private void initializeMotors(int count)
    {
        for (int i = 0; i < count; i++)
        {
            string motorName = "Motor" + i;
            Motor motor = new Motor { Name = motorName };

            List<float> randomValues = new List<float>();
            for (int j = 0; j < 30; j++)
            {
                float value = UnityEngine.Random.Range(0, 100);
                randomValues.Add(value);
            }

            motor.Values = randomValues;
            Motors.Add(motorName, motor);
        }
    }
}

public class Motor
{
    public string Name;
    public List<float> Values = new List<float>();
}
