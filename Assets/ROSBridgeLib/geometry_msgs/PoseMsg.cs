using System.Collections;
using System.Text;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace geometry_msgs
    {
        public class PoseMsg : ROSBridgeMsg
        {
            private PointMsg _position;
            private QuaternionMsg _orientation;

            public PoseMsg(JSONNode msg)
            {
                _position = new PointMsg(msg["position"]);
                _orientation = new QuaternionMsg(msg["orientation"]);
            }

            public PoseMsg(PointMsg position, QuaternionMsg orientation)
            {
                _position = position;
                _orientation = orientation;
            }

            public static string GetMessageType()
            {
                return "geometry_msgs/Pose";
            }

            public PointMsg GetPosition()
            {
                return _position;
            }

            public QuaternionMsg GetOrientation()
            {
                return _orientation;
            }

            public override string ToString()
            {
                return "Pose [position=" + _position.ToString() + ",  orientation=" + _orientation.ToString() + "]";
            }

            public override string ToYAMLString()
            {
                return "{\"position\" : " + _position.ToYAMLString() + ", \"orientation\" : " + _orientation.ToYAMLString() + "}";
            }
        }
    }
}