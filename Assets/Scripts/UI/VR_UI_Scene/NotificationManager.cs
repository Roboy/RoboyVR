using System;
using System.Collections;
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
        yield return new WaitForSeconds(2);
        Notification note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, "head", 3);
        Debug.Log("Added Warning");

        yield return new WaitForSeconds(4);
        note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, "hip", 5);
        Debug.Log("Added Error");

        yield return new WaitForSeconds(2);
        note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.DEBUG, DummyStates.State.MOTOR_DEAD, "foot_left", 5);
        Debug.Log("Added Debug");

        yield return new WaitForSeconds(2);
        note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.DEBUG, DummyStates.State.MOTOR_DEAD, "", 5f);
        Debug.Log("Added debug");
        testing = false;

    }

    /// <summary>
    /// shows 
    /// </summary>
    private void UpdateIconsAndText()
    {
        //Debug.Log("Updating icons and text");
        int[] counts = {
            VRUILogic.Instance.GetAllErrors().Count,
            VRUILogic.Instance.GetAllWarnings().Count,
            VRUILogic.Instance.GetAllDebugs().Count ,
            VRUILogic.Instance.GetAllInfos().Count};

        Texture[] textures = {
            VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.ERROR),
            VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.WARNING),
            VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.DEBUG),
            VRUILogic.Instance.GetIconTexture(DummyStates.MessageType.INFO),
        };

        int slotIndex = 0; //index of the current slot to work with
        //update each slot with current info
        for (int i = 0; i < counts.Length; i++)
        {
            if (counts[i] > 0)
            {
                RawImage currentSlot = m_imageSlots[slotIndex];
                if (!currentSlot.enabled) currentSlot.enabled = true;
                if (!currentSlot.texture.Equals(textures[i]))
                {
                    currentSlot.texture = textures[i];
                }
                Text slotText = currentSlot.GetComponentInChildren<Text>();
                slotText.text = SlotText(counts[i]); //update displayed counts with respective number

                slotIndex++; //work with next slot, this one filled
            }
        }
        //clear rest of the slots
        for (; slotIndex < m_imageSlots.Length; slotIndex++)
        {
            m_imageSlots[slotIndex].enabled = false;
            m_imageSlots[slotIndex].GetComponentInChildren<Text>().text = "";
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
            return String.Empty;
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
