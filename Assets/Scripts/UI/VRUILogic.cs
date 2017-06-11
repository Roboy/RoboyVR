using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VRUILogic : MonoBehaviour {

    #region public database
    public int selectedMode = 0;
    public GameObject[] mode;

    #endregion

    #region private variables
    private static VRUILogic instance;
    #endregion


    /// <summary>
    /// Returns VRUILogic Singleton
    /// </summary>
    /// <returns>Singleton</returns>
    public static VRUILogic GetInstance()
    {
        if (!instance)
        {
            instance = GameObject.FindObjectOfType<VRUILogic>();
        }
        return instance;
    }

    /// <summary>
    /// Creates Singelton
    /// </summary>
    private void Awake()
    {
        if(instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }

    /// <summary>
    /// Starts UI by disabling all except for specified modes.
    /// </summary>
    private void Start()
    {
        foreach(GameObject obj in instance.mode)
        {
            obj.SetActive(false);
        }
        instance.mode[selectedMode].gameObject.SetActive(true);
    }

    /// <summary>
    /// Function displays mode i and disables previous mode.
    /// </summary>
    /// <param name="i">index of mode</param>
    public void SelectedModeChanged(int i)
    {
        if (instance.mode != null)
        {
            Debug.Log("New mode" + i);
            if (i < mode.Length && i >= 0)
            {
                mode[selectedMode].gameObject.SetActive(false);
                mode[i].gameObject.SetActive(true);
                selectedMode = i;
            }
        }
    }

}
