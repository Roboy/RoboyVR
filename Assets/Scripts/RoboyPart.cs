using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
    /// 
    /// </summary>
    public void UpdateHalo()
    {
        if (m_myErrors.Count > 0 || m_myWarnings.Count > 0)
        {
            DisplayErrors();
            DisplayWarnings();
        }else
        {
            DisableErrors();
            DisableWarnings();
        }

    }


    /// <summary>
    /// adds notification to this bodypart. Useful for link between selected body part and respective notifications
    /// </summary>
    /// <param name="note"></param>
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
