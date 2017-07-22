﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlManager : MonoBehaviour, VRUILogic.ISubscriber
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Holds reference to prefab which will be used to create new items in Notification list
    /// </summary>
    [SerializeField]
    private GameObject m_itemPrefab;

    /// <summary>
    /// Reference to the item container of the notification list. 
    /// Items are added to it when instantiated. 
    /// </summary>
    [SerializeField]
    private GameObject m_ListContentContainer;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Awake()
    {
        VRUILogic.Instance.SubscribeNotifications(this);
    }

    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {

    }
    #endregion

    #region PUBLIC_METHODS
    #region subscriber methods    
    /// <summary>
    /// when notification deleted, it will automatically delete the respective item -> do nothing
    /// </summary>
    public void BeInformed()
    {
    }

    /// <summary>
    /// When informed of new notification -> add item to list
    /// </summary>
    /// <param name="info"></param>
    public void BeInformed(object info)
    {
        if (info.GetType().Equals(typeof(Notification)))
        {
            GameObject newItem = Instantiate(m_itemPrefab);
            newItem.GetComponent<NotificationListButton>().SetupItem((Notification)info);
            newItem.transform.SetParent(m_ListContentContainer.transform);
            newItem.transform.localScale = Vector3.one;
        }
    }
    #endregion
    #endregion

    #region PRIVATE_METHODS
    #endregion
}
