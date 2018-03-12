using SimpleJSON;
using UnityEngine;


namespace ROSBridgeLib
{
    namespace custom_msgs
    {
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
                _linkname = msg["linkname"];
                _position.x = float.Parse(msg["x"]);
                _position.y = float.Parse(msg["y"]);
                _position.z = float.Parse(msg["z"]);

                _force.x = float.Parse(msg["x"]);
                _force.y = float.Parse(msg["y"]);
                _force.z = float.Parse(msg["z"]);

                _duration = int.Parse(msg["duration"]);
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
                //remainder from prev implementation below
                //return "ExternalForce [name=" + _linkname + ", position=" + _position.ToString() + ",  force=" + _force.ToString() + ", duration=" + _duration.ToString() + "]";
            }

            public override string ToYAMLString()
            {
                return "{" + "\"name\" : \"" + _linkname + "\", " + // name
                    "\"x\" : " + _position.x + ", \"y\" : " + _position.y + ", \"z\" : " + _position.z + ", " + //pos
                    "\"f_x\" : " + _position.x + ", \"f_y\" : " + _position.y + ", \"f_z\" : " + _position.z + ", " + //force 
                    "\"duration\" : " + _duration + "}"; // duration
            }
        }
    }
}
