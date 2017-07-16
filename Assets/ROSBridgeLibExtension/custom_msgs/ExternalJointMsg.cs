using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.std_msgs;
using SimpleJSON;
using UnityEngine;


namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class ExternalJointMsg : ROSBridgeMsg
        {
            public custom_msgs.StringArrayMsg JointNames
            {
                get
                {
                    return _jointNames;
                }
            }

            public custom_msgs.FloatArrayMsg Angles
            {
                get
                {
                    return _angles;
                }
            }

            private custom_msgs.StringArrayMsg _jointNames;
            private custom_msgs.FloatArrayMsg _angles;

            public ExternalJointMsg(JSONNode msg)
            {
            //TODO implement in the future
            }

            public ExternalJointMsg(List<string> jointNames, List<float> angles)
            {
                _jointNames = new StringArrayMsg("link_name", jointNames);
                _angles = new FloatArrayMsg("angle", angles);
            }

            public static string GetMessageType()
            {
                return "roboy_communication_middleware/JointCommandRevolute";
            }

            public override string ToString()
            {
                return "roboy_communication_middleware/JointCommandRevolute [name=";
            }

            public override string ToYAMLString()
            {
                return "{" + _jointNames.ToYAMLString() + ", " + _angles.ToYAMLString() + "}";
            }
        }
    }
}
