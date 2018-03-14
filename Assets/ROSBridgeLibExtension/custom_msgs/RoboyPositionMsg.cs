using SimpleJSON;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Message containing x,y,z coordinates of a roboy model: Not specified, which model (if desired)
        /// Accounts for parsing errors. 
        /// </summary>
        public class RoboyPositionMsg : ROSBridgeMsg
        {
            #region PUBLIC_MEMBER_VARIABLES
            public Vector3 position
            {
                get
                {
                    return _position;
                }
            }
            #endregion

            #region PRIVATE_MEMBER_VARIABLES
            /// <summary>
            /// position, can be either unity or gazebo coord space
            /// </summary>
            private Vector3 _position;

            private bool inUnityCoordSPace;
            #endregion

            #region PUBLIC_METHODS
            /// <summary>
            /// Parses received message and transforms gazebo coordinates into Unity coordinate system
            /// TODO: not tested yet
            /// </summary>
            /// <param name="msg"></param>
            public RoboyPositionMsg(JSONNode msg)
            {
                //TODO implement in the future
                Vector3 gazeboPos = Vector3.zero;
                if (!float.TryParse(msg["x"], out _position.x) || !float.TryParse(msg["y"], out _position.y) || !float.TryParse(msg["z"], out _position.z))
                {
                    Debug.LogWarning("Received malformed RoboyPositionMsg: received values could not be parsed to float");
                }
                Vector3 unityPos = GazeboUtility.GazeboPositionToUnity(gazeboPos);
                inUnityCoordSPace = true;
            }

            /// <summary>
            /// Creates a message containing the given position and TRANSFORMS IT INTO GAZEBO COORDINATE SYSTEM 
            /// </summary>
            /// <param name="position">position in UNITY COORD SPACE</param>
            public RoboyPositionMsg(Vector3 position)
            {
                _position = GazeboUtility.UnityPositionToGazebo(position);
            }

            public static string GetMessageType()
            {
                return "roboy_communication_middleware/Position";
            }

            public override string ToString()
            {
                throw new System.NotImplementedException();
            }

            public override string ToYAMLString()
            {
                return "{" + "\"x\" : " + _position.x + ", \"y\" : " + _position.y + ", \"z\" : " + _position.z + "}";
            }
            #endregion
        }
    }
}
