using UnityEngine;

/// <summary>
/// Script manages order and displayed content of tabs.
/// Attached to screen. Methods should be called once one tab selected.
/// </summary>
public class ScreenTabManager : MonoBehaviour
{

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// All tabs belonging to the screen
    /// </summary>
    [SerializeField]
    private GameObject[] m_Tabs;
    #endregion

    /// <summary>
    /// When Screen enabled, update tab order
    /// </summary>
    void OnEnable()
    {
        OnTabOrderChanged();
    }

    /// <summary>
    /// can be called by tabs when OnClick Event triggered, adjusts the displayed elements
    /// Relies on the fact that clicked tab set its SiblingIndex to last one (be rendered last, displayed over all)
    /// </summary>
    #region PUBLIC_METHODS
    public void OnTabOrderChanged()
    {
        foreach (GameObject tab in m_Tabs)
        {
            GameObject panel = tab.transform.Find("Panel").gameObject;
            if (tab.transform.GetSiblingIndex() < m_Tabs.Length - 1)
            {
                panel.SetActive(false);
            }
            else
            {
                panel.SetActive(true);
            }
        }
    }
    #endregion
}

