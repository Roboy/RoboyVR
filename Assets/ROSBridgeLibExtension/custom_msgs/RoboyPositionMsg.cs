using SimpleJSON;
using UnityEngine;
/// <summary>
/// Message containing x,y,z coordinates of a roboy model: Not specified, which model (if desired)
/// </summary>
namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class RoboyPositionMsg : ROSBridgeMsg
        {
            public float x
            {
                get
                {
                    return _x;
                }
            }

            public float y
            {
                get
                {
                    return _y;
                }
            }

            public float z
            {
                get
                {
                    return _z;
                }
            }

            private float _x;
            private float _y;
            private float _z;


            /// <summary>
            /// Parses received message and transforms gazebo coordinates into Unity coordinate system
            /// TODO: not tested yet
            /// </summary>
            /// <param name="msg"></param>
            public RoboyPositionMsg(JSONNode msg)
            {
                //TODO implement in the future
                Vector3 gazeboPos = new Vector3(float.Parse(msg["x"]), float.Parse(msg["y"]), float.Parse(msg["z"]));
                Vector3 unityPos = GazeboUtility.GazeboPositionToUnity(gazeboPos);
                _x = unityPos.x;
                _y = unityPos.y;
                _z = unityPos.z;
            }

            /// <summary>
            /// Creates a message containing the given position and TRANSFORMS IT INTO GAZEBO COORDINATE SYSTEM 
            /// </summary>
            /// <param name="x"></param>
            /// <param name="y"></param>
            /// <param name="z"></param>
            public RoboyPositionMsg(float x, float y, float z)
            {
                Vector3 tmp = GazeboUtility.UnityPositionToGazebo(new Vector3((float)x, (float)y, (float)z));
                _x = tmp.x;
                _y = tmp.y;
                _z = tmp.z;

            }

            public static string GetMessageType()
            {
                return "roboy_communication_middleware/Position";
            }

            public override string ToString()
            {
                return "roboy_communication_middleware/Position [name=";
            }

            public override string ToYAMLString()
            {
                return "{" + "\"x\" : " + _x + ", \"y\" : " + _y + ", \"z\" : " + _z + "}";
            }
        }
    }
}
