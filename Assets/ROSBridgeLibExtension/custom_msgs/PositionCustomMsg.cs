using SimpleJSON;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class PositionCustomMsg : ROSBridgeMsg
        {
            private float _x;
            private float _y;
            private float _z;

            public PositionCustomMsg(JSONNode msg)
            {
                Vector3 gazeboPos = new Vector3(float.Parse(msg["x"]), float.Parse(msg["y"]), float.Parse(msg["z"]));
                Vector3 unityPos = GazeboUtility.GazeboPositionToUnity(gazeboPos);
                _x = unityPos.x;
                _y = unityPos.y;
                _z = unityPos.z;
            }
            
            /// <summary>
            /// Constructor for position msg: Position in unity coord space
            /// </summary>
            /// <param name="position"></param>
            public PositionCustomMsg(Vector3 position)
            {
                Vector3 gazebopos = GazeboUtility.UnityPositionToGazebo(position);
                _x = gazebopos.x;
                _y = gazebopos.y;
                _z = gazebopos.z;
            }

            public static string GetMessageType()
            {
                return "geometry_msgs/Vector3";
            }

            public float GetX()
            {
                return _x;
            }

            public float GetY()
            {
                return _y;
            }

            public float GetZ()
            {
                return _z;
            }

            public override string ToString()
            {
                return "Force [fx=" + _x + ",  fy=" + _y + ",  fz=" + _z + "]";
            }

            public override string ToYAMLString()
            {
                return "\"x\" : " + _x + ", \"y\" : " + _y + ", \"z\" : " + _z;
            }
        }
    }
}
