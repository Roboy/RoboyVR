using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackingManager : MonoBehaviour {

    /// <summary>
    /// Turn head tracking for BeRoboy on.
    /// </summary>
    public void TurnTrackingOn()
    {
        BeRoboyManager.Instance.TrackingEnabled = true;
    }


    /// <summary>
    /// Turn head tracking for BeRoboy off.
    /// </summary>
    public void TurnTrackingOff()
    {
        BeRoboyManager.Instance.TrackingEnabled = false;
    }
}
