using SimpleJSON;
using UnityEngine;


namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// External force msg consisting of concerned link, local position, local force(dir + strength), duration
        /// Returns error if parsing of float values fails
        /// </summary>
        public class ExternalForceMsg : ROSBridgeMsg
        {
            /// <summary>
            /// name of the affected link
            /// </summary>
            private string _linkname;

            /// <summary>
            /// position of the force in local space of link
            /// </summary>
            private Vector3 _position;

            /// <summary>
            /// direction of force in local space of link
            /// </summary>
            private Vector3 _force;

            /// <summary>
            /// duration in milliseconds
            /// </summary>
            private int _duration;

            /// <summary>
            /// If set to true, this message is outgoing -> values are in gazebo coord system
            /// </summary>
            private bool publish; 


            /// <summary>
            /// Creates message: normally called when receiving message on subscription topic
            /// </summary>
            /// <param name="msg"></param>
            public ExternalForceMsg(JSONNode msg)
            {
                //if any of these parsing operations does not succeed
                if(!float.TryParse(msg["x"],out _position.x) || !float.TryParse(msg["y"], out _position.y) ||
                    !float.TryParse(msg["z"], out _position.z) || !float.TryParse(msg["f_x"], out _force.x)
                    || !float.TryParse(msg["f_y"], out _force.y) || !float.TryParse(msg["f_z"], out _force.z)
                    || !int.TryParse(msg["duration"], out _duration)
                    )
                {
                    Debug.LogWarning("Received malformed message");
                    return;
                }
                _linkname = msg["linkname"];
                //incoming message -> parse from gazebo coordinate system to unity's
                publish = false;
                _position = GazeboUtility.GazeboPositionToUnity(_position);
                _force = GazeboUtility.GazeboPositionToUnity(_force);
            }

            /// <summary>
            /// Creates message specifying given external force.
            /// NOTICE: COORDINATE SYSTEM TRANSLATION HAPPENS HERE -> set publish to true
            /// </summary>
            /// <param name="linkname">Name of affected link</param>
            /// <param name="position">position in UNity coordinate space</param>
            /// <param name="force">force in Unity coordinate space</param>
            /// <param name="duration"></param>
            /// <param name="publish">If set to true, this message is outgoing -> will be parsed to gazebo coord system</param>
            public ExternalForceMsg(string linkname, Vector3 position, Vector3 force, int duration, bool publish)
            {
                _linkname = linkname;
                _position = position;
                _force = force;
                _duration = duration;
                //outgoing message-> parse to gazebo's coordinate system
                if (publish)
                {
                    _position = GazeboUtility.UnityPositionToGazebo(_position);
                    _force = GazeboUtility.UnityPositionToGazebo(_force);
                }
            }

            /// <summary>
            /// Returns whether this message's coordinates are defined in Unity's coordinate system or Gazebo's.
            /// </summary>
            /// <returns></returns>
            public bool IsInUnityCoordinateSystem()
            {
                return !publish;
            }

            public static string GetMessageType()
            {
                return "roboy_communication_simulation/ExternalForce";
            }

            /// <summary>
            /// TODO: never used before.... is this needed ? for now: not implemented exception!
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                throw new System.NotImplementedException();
            }

            public override string ToYAMLString()
            {
                return "{" + "\"name\" : \"" + _linkname + "\", " + // name
                    "\"x\" : " + _position.x + ", \"y\" : " + _position.y + ", \"z\" : " + _position.z + ", " + //pos
                    "\"f_x\" : " + _force.x + ", \"f_y\" : " + _force.y + ", \"f_z\" : " + _force.z + ", " + //force 
                    "\"duration\" : " + _duration + "}"; // duration
            }
        }
    }
}
