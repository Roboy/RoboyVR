using UnityEngine;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class ForceMsg : ROSBridgeMsg
        {
            private float _x;
            private float _y;
            private float _z;

            public ForceMsg(JSONNode msg)
            {
                Vector3 gazeboPos = new Vector3(float.Parse(msg["f_x"]), float.Parse(msg["f_y"]), float.Parse(msg["f_z"]));
                Vector3 unityPos = GazeboUtility.GazeboPositionToUnity(gazeboPos);
                _x = unityPos.x;
                _y = unityPos.y;
                _z = unityPos.z;
            }

            /// <summary>
            /// Constructor for force msg: Expects force in unity coordinate system and transforms it to Gazebo coordinate system
            /// </summary>
            /// <param name="force"></param>
            public ForceMsg(Vector3 force)
            {
                Vector3 gazeboForce = GazeboUtility.UnityPositionToGazebo(force);
                _x = gazeboForce.x;
                _y = gazeboForce.y;
                _z = gazeboForce.z;
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
                return "\"f_x\" : " + _x + ", \"f_y\" : " + _y + ", \"f_z\" : " + _z;
            }
        }
    }
}
