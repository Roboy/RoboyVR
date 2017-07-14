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
        public class ExternalForceMsg : ROSBridgeMsg
        {
            public custom_msgs.LinkMsg Linkname
            {
                get
                {
                    return _linkname;
                }
            }

            public custom_msgs.PositionCustomMsg Position
            {
                get
                {
                    return _position;
                }
            }

            public custom_msgs.ForceMsg Force
            {
                get
                {
                    return _force;
                }
            }

            public custom_msgs.DurationMsg Duration
            {
                get
                {
                    return _duration;
                }
            }

            private custom_msgs.LinkMsg _linkname;
            private custom_msgs.PositionCustomMsg _position;
            private custom_msgs.ForceMsg _force;
            private custom_msgs.DurationMsg _duration;



            public ExternalForceMsg(JSONNode msg)
            {
                _linkname = new custom_msgs.LinkMsg(msg["linkname"]);
                _position = new custom_msgs.PositionCustomMsg(msg["position"]);
                _force = new custom_msgs.ForceMsg(msg["force"]);
                _duration = new custom_msgs.DurationMsg(msg["duration"]);
            }

            public ExternalForceMsg(custom_msgs.LinkMsg linkname, custom_msgs.PositionCustomMsg position, custom_msgs.ForceMsg force, custom_msgs.DurationMsg duration)
            {
                _linkname = linkname;
                _position = position;
                _force = force;
                _duration = duration;
            }

            public ExternalForceMsg(string linkname, Vector3 position, Vector3 force, int duration)
            {
                _linkname = new LinkMsg(linkname);
                _position = new PositionCustomMsg(position.x, position.y, position.z);
                _force = new ForceMsg(force.x, force.y, force.z);
                _duration = new DurationMsg(duration);
            }

            public static string GetMessageType()
            {
                return "common_utilities/ExternalForce";
            }

            public override string ToString()
            {
                return "ExternalForce [name=" + _linkname + ", position=" + _position.ToString() + ",  force=" + _force.ToString() + ", duration=" + _duration.ToString() + "]";
            }

            public override string ToYAMLString()
            {
                return "{" + _linkname.ToYAMLString() + ", " + _position.ToYAMLString() + ", " + _force.ToYAMLString() + ", " + _duration.ToYAMLString() + "}";
            }
        }
    }
}
