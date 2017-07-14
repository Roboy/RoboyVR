using ROSBridgeLib;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// All ROSObjects tell the ROSBridge to add them when they are enabled and to remove them when they are disabled from the connection.
/// </summary>
public class ROSObject : MonoBehaviour {

    private void OnEnable()
    {
        ROSBridge.Instance.AddROSActor(this);
    }

    private void OnDisable()
    {
        // we need the check as ROSBridge is a singleton and we destroy it if the application is quitting.
        if(ROSBridge.Instance != null)
            ROSBridge.Instance.RemoveROSActor(this);
    }
}
