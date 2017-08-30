using System;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// This class is attached to the canvas, which is always displayed, as some notifications (their icons and halos) are always displayed.
/// Different settings depending on the mode are considered (for now no changes)
/// Subscriber Interface needed to be notified of updates. 
/// </summary>
public class NotificationManager : MonoBehaviour, VRUILogic.ISubscriber
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Icon(rawimage) slot
    /// Image must contain child text object
    /// </summary>
    [SerializeField]
    private RawImage[] m_ImageSlots;

    /// <summary>
    /// reference to screen which displays slots. If attached, it is enabled upon start
    /// </summary>
    [SerializeField]
    private Canvas m_Screen;

    /// <summary>
    /// saves the current mode setting, in which the notification manager is
    /// </summary>
    private VRUILogic.UIMode m_CurrentModeSetting;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// sets and manages stuff that is always displayed 
    /// </summary>
    public void OnEnable()
    {
        if (m_Screen)
            m_Screen.enabled = true;
        VRUILogic.Instance.SubscribeNotifications(this);
        UpdateIconsAndText();
    }

    /// <summary>
    /// updates displayed notifications if changes in mode occured
    /// </summary>
    void Update()
    {

        VRUILogic.UIMode newMode = VRUILogic.Instance.GetCurrentMode();
        if (newMode != m_CurrentModeSetting)
        {
            m_CurrentModeSetting = newMode;
            //only in control mode warnings are displayed
            if (m_CurrentModeSetting == VRUILogic.UIMode.Control)
            {
                //TODO: Display warnings in this mode
                foreach (Notification warning in VRUILogic.Instance.GetAllWarnings())
                {
                    warning.DisplayHalo();
                }
            }
            else
            {
                //TODO: hide warnings-> not working yet ...not needed?
                foreach (Notification warning in VRUILogic.Instance.GetAllWarnings())
                {
                    //warning.DisableHalo();
                }
            }
        }
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion


    #region PRIVATE_METHODS

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
                RawImage currentSlot = m_ImageSlots[slotIndex];
                if (!currentSlot.enabled) currentSlot.enabled = true;
                if (!currentSlot.texture.Equals(textures[i]))
                {
                    currentSlot.texture = textures[i];
                }
                Text slotText = currentSlot.GetComponentInChildren<Text>();
                slotText.text = GetSlotText(counts[i]); //update displayed counts with respective number

                slotIndex++; //work with next slot, this one filled
            }
        }
        //clear rest of the slots
        for (; slotIndex < m_ImageSlots.Length; slotIndex++)
        {
            m_ImageSlots[slotIndex].enabled = false;
            m_ImageSlots[slotIndex].GetComponentInChildren<Text>().text = "";
        }
    }
    /// <summary>
    /// sets respective text for slot depending on count 
    /// </summary>
    /// <param name="count">count which is to be transformed into text form</param>
    private string GetSlotText(int count)
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
    /// receives updates on notifications (from database)
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
    /// called when notifications deleted. if no arguments passed, elems were deleted -> update
    /// </summary>
    public void BeInformed()
    {
        UpdateIconsAndText();
    }
    #endregion
}
