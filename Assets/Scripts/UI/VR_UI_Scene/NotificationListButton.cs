using UnityEngine;
using UnityEngine.UI;

public class NotificationListButton : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// reference to the text displaying the RoboyPartName and state in string form
    /// </summary>
    [SerializeField]
    private Text m_BasicInformation;

    /// <summary>
    /// Button component instance of this gameObject
    /// </summary>
    [SerializeField]
    private Button m_Button;

    /// <summary>
    /// Holds a reference to the prefab that is instantiated to display additional information OnClick
    /// </summary>
    [SerializeField]
    private GameObject m_AdditionalNotificationInfoPrefab;

    /// <summary>
    /// link to the image which displays the icon of this list element.
    /// </summary>
    [SerializeField]
    private RawImage m_Icon;

    /// <summary>
    /// reference needed to display additional information
    /// </summary>
    private Notification m_note;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    void OnButtonClick()
    {

    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// This function sets all values and references according to the notification it is supposed to represent. 
    /// </summary>
    /// <param name="note">The notification this listItem will represent</param>
    public void SetupItem(Notification note)
    {
        m_BasicInformation.text = note.GetBasicInfo();
        m_Icon.texture = VRUILogic.Instance.GetIconTexture(note.GetNotificationType());
        //links this item to notification
        //ensures that this list item is deleted as soon as notification is outdated
        note.AddRelatedObject(gameObject);
        m_note = note;


        //TODO: must be set, but rect.width is zero due to automatic scaling scripts... 

        //BoxCollider collider = GetComponent<BoxCollider>();
        //Rect rect = GetComponent<RectTransform>().rect;//no way to access the value found as of now... set manually
        //collider.size = new Vector3(rect.width, rect.height, 0.01f);
        //Debug.Log("UI BUTTON SIZE: " + collider.size);
    }

    /// <summary>
    /// Called as soon as event is triggered. Instantiates new panel with additional information
    /// </summary>
    public void OnClick()
    {
        //prevents multiple instances of this screen (issues with buttons otherwise)
        GameObject infoScreen = GameObject.Find("AdditionalInfoScreen");
        if (!infoScreen)
        {
            infoScreen = Instantiate(m_AdditionalNotificationInfoPrefab);
            infoScreen.name = "AdditionalInfoScreen";
        }
        infoScreen.GetComponentInChildren<AdditionalInfoScreenManager>().Setup(m_note);
    }
    #endregion
}

