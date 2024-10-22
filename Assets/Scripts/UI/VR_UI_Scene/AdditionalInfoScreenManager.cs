﻿using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// This script manages the functionality of the additional screen.
/// It is attached to the gameobject and sets all text and image components and reacts to click events. 
/// </summary>
public class AdditionalInfoScreenManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// Image containing icon
    /// </summary>
    [SerializeField]
    private RawImage m_Icon;

    /// <summary>
    /// Text displayed as headline. 
    /// -> Warning/Debug/info/error
    /// </summary>
    [SerializeField]
    private Text m_Headline;

    /// <summary>
    /// text displaying  the body part
    /// </summary>
    [SerializeField]
    private Text m_BodyPartText;

    /// <summary>
    /// text displaying the state
    /// </summary>
    [SerializeField]
    private Text m_StateText;

    /// <summary>
    /// text displaying more content if existing
    /// </summary>
    [SerializeField]
    private Text m_MoreContent;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS##
    /// <summary>
    /// Called as soon as event (button click) is triggered. deletes this extra info screen
    /// </summary>
    public void OnClick()
    {
        Destroy(gameObject);
    }
    #endregion

    #region PUBLIC_METHODS

    /// <summary>
    /// populates Additional Information screen with respective values / settings
    /// </summary>
    /// <param name="note">Note for which this screen is displayed</param>
    public void Setup(Notification note)
    {
        if (note)
        {
            m_Icon.texture = VRUILogic.Instance.GetIconTexture(note.GetNotificationType());
            m_Headline.text = note.GetNotificationType().ToString();
            GameObject part = note.GetConcernedRoboyPart();
            if (part)
                m_BodyPartText.text = part.name;
            else
                m_BodyPartText.text = "[General]";
            m_StateText.text = note.GetState().ToString();
            m_MoreContent.text = note.GetAdditionalInfo();
        }
        GetComponent<Canvas>().worldCamera = VRUILogic.Instance.GetCamera();
    }
    #endregion
}

