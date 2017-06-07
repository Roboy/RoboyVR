using System;
using ROSBridgeLib;
using SimpleJSON;
using UnityEngine;
using ROSBridgeLib.sensor_msgs;

public class RoboyCameraSubscriber : ROSBridgeSubscriber
{

    #region PUBLIC_MEMBER_VARIABLES
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public new static string GetMessageTopic()
    {
        return "/roboy/camera/image_raw";

    }

    public new static string GetMessageType()
    {
        return "sensor_msgs/Image";

    }

    public new static ROSBridgeMsg ParseMessage(JSONNode msg)
    {
        //ImageMsg from sensor messages lib
        return new ImageMsg(msg);
    }

    public new static void CallBack(ROSBridgeMsg msg)
    {
        ImageMsg image = (ImageMsg)msg;
        BeRoboyManager.Instance.ReceiveMessage(image);

    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion //PRIVATE_METHODS
}
