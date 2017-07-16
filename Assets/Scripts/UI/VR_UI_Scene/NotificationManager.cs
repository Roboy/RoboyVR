using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// This class is attached to  the canvas, which is always displayed, as some notifications are always displayed.
/// Depending on some modes, additional / less information is shown. 
/// </summary>
public class NotificationManager : MonoBehaviour, VRUILogic.ISubscriber
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Screen Overlay canvas on which to display the notifications
    /// </summary>
    private Canvas canvas;

    /// <summary>
    /// Icon(rawimage) slot
    /// Image must contain child text object
    /// </summary>
    [SerializeField]
    private RawImage[] m_imageSlots;

    /// <summary>
    /// Texture of error icon
    /// </summary>
    [SerializeField]
    private Texture m_error;
    /// <summary>
    /// texture of warning icon
    /// </summary>
    [SerializeField]
    private Texture m_warning;
    /// <summary>
    /// texture of debug icon
    /// </summary>
    [SerializeField]
    private Texture m_debug;
    /// <summary>
    /// saves the current mode setting, in which the notification manager is
    /// </summary>
    private VRUILogic.UIMode m_currentModeSetting;

    /// <summary>
    /// boolean if test coroutine running
    /// </summary>
    private bool testing = false;
    #endregion
    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// sets and manages stuff that is always displayed 
    /// </summary>
    public void OnEnable()
    {
        VRUILogic.Instance.SubscribeNotifications(this);
        UpdateIconsAndText();
    }

    /// <summary>
    /// updates displayed notifications if changes occured
    /// </summary>
    void Update()
    {
        //test notifications display
        if (!testing)
        {
            testing = true;
            StartCoroutine(test());
            Debug.Log("Notification test started");
        }

        VRUILogic.UIMode newMode = VRUILogic.Instance.GetCurrentMode();
        if (newMode != m_currentModeSetting)
        {
            m_currentModeSetting = newMode;
            //only in control mode warnings are displayed
            if (m_currentModeSetting == VRUILogic.UIMode.Control)
            {
                //TODO: Display warnings
                foreach (Notification warning in VRUILogic.Instance.GetAllWarnings())
                {
                    warning.DisplayHalo();
                }
            }
            else
            {
                //TODO: hide warnings-> not working yet
                foreach (Notification warning in VRUILogic.Instance.GetAllWarnings())
                {
                    warning.DisplayHalo();
                }
            }
        }
    }
    #endregion
    #region PUBLIC_METHODS
    #endregion


    #region PRIVATE_METHODS
    /// <summary>
    /// used for testing of notifications display
    /// </summary>
    /// <returns></returns>
    private IEnumerator test()
    {
        Notification note = new Notification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, 0);
        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNotification(note);

        yield return new WaitForSeconds(2);
        note = new Notification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, 0);
        VRUILogic.Instance.AddNotification(note);
        Debug.Log("Warning");
        yield return new WaitForSeconds(2);
        note = new Notification(DummyStates.MessageType.DEBUG, DummyStates.State.MOTOR_DEAD, 0);
        VRUILogic.Instance.AddNotification(note);
        Debug.Log("debug");

        yield return new WaitForSeconds(2);
        VRUILogic.Instance.ClearAllNotifications();
        Debug.Log("Clear all notifications");

        yield return new WaitForSeconds(2);
        note = new Notification(DummyStates.MessageType.DEBUG, DummyStates.State.MOTOR_DEAD, 0);
        VRUILogic.Instance.AddNotification(note);
        Debug.Log("debug");
        testing = false;
    }

    /// <summary>
    /// shows 
    /// </summary>
    private void UpdateIconsAndText()
    {
        Debug.Log("Updating icons and text");
        int errorCount = VRUILogic.Instance.GetAllErrors().Count;
        int warningCount = VRUILogic.Instance.GetAllWarnings().Count;
        int debugCount = VRUILogic.Instance.GetAllDebugs().Count;
        int count = 0;
        //update each slot with current info
        if (errorCount > 0)
        {
            Debug.Log("Updating first slot");

            if (!m_imageSlots[count].enabled) m_imageSlots[count].enabled = true;
            Texture current = m_imageSlots[count].texture;
            if (current != m_error)
            {
                Debug.Log("Setting Texture");
                m_imageSlots[count].texture = m_error;
            }
            Text slotText = m_imageSlots[count].GetComponentInChildren<Text>();
            slotText.text = SlotText(errorCount);
            count++;
        }
        if (warningCount > 0)
        {
            Debug.Log("Updating second slot");

            if (!m_imageSlots[count].enabled) m_imageSlots[count].enabled = true;
            Texture current = m_imageSlots[count].texture;
            if (current != m_warning) m_imageSlots[count].texture = m_warning;
            Text slotText = m_imageSlots[count].GetComponentInChildren<Text>();
            slotText.text = SlotText(warningCount);
            count++;
        }
        if (debugCount > 0)
        {
            Debug.Log("Updating third slot");

            if (!m_imageSlots[count].enabled) m_imageSlots[count].enabled = true;
            Texture current = m_imageSlots[count].texture;
            if (current != m_debug) m_imageSlots[count].texture = m_debug;
            Text slotText = m_imageSlots[count].GetComponentInChildren<Text>();
            slotText.text = SlotText(debugCount);
            count++;
        }
        //clear rest of the slots
        for (; count < 3; count++)
        {
            Debug.Log("disabling rest");

            m_imageSlots[count].enabled = false;
            m_imageSlots[count].GetComponentInChildren<Text>().text = "";
        }
    }

    /// <summary>
    /// sets respective text for slot depending on count 
    /// </summary>
    /// <param name="count">count which is to be transformed into text form</param>
    private string SlotText(int count)
    {
        if (count == 0)
        {
            Debug.Log("[NotificationManager] Something went wrong!!!");
            return ": none";
        }
        if (count == 1)
        {
            return "";
        }
        return "(" + count + ")";
    }

    /// <summary>
    /// receives changes to notifications
    /// </summary>
    /// <param name="info"></param>
    public void BeInformed(object info)
    {
        if (info.GetType().Equals(typeof(Notification)))
        {
            ((Notification)info).DisplayHalo();
            UpdateIconsAndText();
        }
    }

    /// <summary>
    /// called when notifications deleted. if no arguments passed, no halos will be created
    /// </summary>
    public void BeInformed()
    {
        UpdateIconsAndText();
    }
    #endregion
}
