using System.Collections.Generic;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Msg consisting of a list of joint angles and its values, only outgoing side implemented. 
        /// Not used as of now
        /// </summary>
        public class ExternalJointMsg : ROSBridgeMsg
        {
            public StringArrayMsg JointNames
            {
                get
                {
                    return _jointNames;
                }
            }

            public FloatArrayMsg Angles
            {
                get
                {
                    return _angles;
                }
            }

            private StringArrayMsg _jointNames;
            private FloatArrayMsg _angles;

            /// <summary>
            /// parses received messages to given format
            /// </summary>
            /// <param name="msg"></param>
            public ExternalJointMsg(JSONNode msg)
            {
                throw new System.NotImplementedException();
            }

            public ExternalJointMsg(List<string> jointNames, List<float> angles)
            {
                _jointNames = new StringArrayMsg("link_name", jointNames);
                _angles = new FloatArrayMsg("angle", angles);
            }

            public static string GetMessageType()
            {
                return "roboy_communication_middleware/JointCommand";
            }

            public override string ToString()
            {
                throw new System.NotImplementedException();
            }

            public override string ToYAMLString()
            {
                return "{" + _jointNames.ToYAMLString() + ", " + _angles.ToYAMLString() + "}";
            }
        }
    }
}
