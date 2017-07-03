using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenTabManager : MonoBehaviour {

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// All tabs belonging to the screen
    /// </summary>
    [SerializeField]
    private GameObject[] tabs;
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
    /// </summary>
    #region PUBLIC_METHODS
    public void OnTabOrderChanged()
    {
        foreach (GameObject tab in tabs)
        {
            GameObject panel = tab.transform.Find("Panel").gameObject;
            if (tab.transform.GetSiblingIndex() < tabs.Length-1)
            {
                panel.SetActive(false);
            }else
            {
                panel.SetActive(true);
            }
        }
    }
    #endregion
}

