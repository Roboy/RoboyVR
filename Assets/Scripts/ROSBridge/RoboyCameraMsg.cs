using System.Collections.Generic;
using SimpleJSON;
using UnityEngine;

public class RoboyCameraMsg : ROSBridgeMsg
{

    #region PUBLIC_MEMBER_VARIABLES
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public RoboyCameraMsg(JSONNode msg)
    {
       //do something here
    }

    public RoboyCameraMsg(string name, Vector3 position, Quaternion rotation)
    {
        //_meshName = name;
        //_position = position;
        //_rotation = rotation;
    }

    public static string GetMessageType()
    {
        return "sensor_msgs/Image";
    }

    public override string ToString()
    {
        return "sensor_msgs/Image [name =";
    }

    public override string ToYAMLString()
    {
        ROSBridgeLib.std_msgs.TimeMsg timeMsg = new ROSBridgeLib.std_msgs.TimeMsg(0, 0);
        ROSBridgeLib.std_msgs.HeaderMsg header = new ROSBridgeLib.std_msgs.HeaderMsg(0, timeMsg, "'world'");
        ROSBridgeLib.std_msgs.StringMsg nameSpace = new ROSBridgeLib.std_msgs.StringMsg("''");
        ROSBridgeLib.std_msgs.Int32Msg id = new ROSBridgeLib.std_msgs.Int32Msg(0);
        ROSBridgeLib.std_msgs.Int32Msg type = new ROSBridgeLib.std_msgs.Int32Msg(10);
        ROSBridgeLib.std_msgs.Int32Msg action = new ROSBridgeLib.std_msgs.Int32Msg(0);
        ROSBridgeLib.geometry_msgs.PointMsg position = new ROSBridgeLib.geometry_msgs.PointMsg(0d, 0d, 0d);
        ROSBridgeLib.geometry_msgs.QuaternionMsg orientation = new ROSBridgeLib.geometry_msgs.QuaternionMsg(0d, 0d, 0d, 0d);
        ROSBridgeLib.geometry_msgs.PoseMsg pose = new ROSBridgeLib.geometry_msgs.PoseMsg(position, orientation);
        ROSBridgeLib.geometry_msgs.Vector3Msg scale = new ROSBridgeLib.geometry_msgs.Vector3Msg(1d, 1d, 1d);
        ROSBridgeLib.std_msgs.ColorRGBAMsg color = new ROSBridgeLib.std_msgs.ColorRGBAMsg(0f, 0f, 0f, 0f);
        ROSBridgeLib.std_msgs.TimeMsg lifetime = new ROSBridgeLib.std_msgs.TimeMsg(10, 0);
        ROSBridgeLib.std_msgs.BoolMsg frame_locked = new ROSBridgeLib.std_msgs.BoolMsg(false);
        ROSBridgeLib.geometry_msgs.PointMsg point = new ROSBridgeLib.geometry_msgs.PointMsg(0d, 0d, 0d);
        ROSBridgeLib.std_msgs.StringMsg mesh_resource = new ROSBridgeLib.std_msgs.StringMsg("'package://roboy_models/legs_with_upper_body/cad/torso.STL'");
        ROSBridgeLib.std_msgs.BoolMsg mesh_use_materials = new ROSBridgeLib.std_msgs.BoolMsg(true);

        return "{" + header.ToYAMLString() + "}";

        //return "{" + header.ToYAMLString() + nameSpace.ToYAMLString() + id.ToYAMLString() + type.ToYAMLString()
        //    + action.ToYAMLString() + pose.ToYAMLString() + scale.ToYAMLString() + color.ToYAMLString()
        //    + lifetime.ToYAMLString() + frame_locked.ToYAMLString() + "{\"points\" : - " + point.ToYAMLString() + "}" + "{\"colors\" : - " + color.ToYAMLString() + "}"
        //    + nameSpace.ToYAMLString() + mesh_resource.ToYAMLString() + mesh_use_materials.ToYAMLString() + "}";
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion //PRIVATE_METHODS
}
