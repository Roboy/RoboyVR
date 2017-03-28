using System.Collections;
using System.Collections.Generic;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class ForceMsg : ROSBridgeMsg
        {
            private double _x;
            private double _y;
            private double _z;

            public ForceMsg(JSONNode msg)
            {
                _x = double.Parse(msg["f_x"]);
                _y = double.Parse(msg["f_y"]);
                _z = double.Parse(msg["f_z"]);
            }

            public ForceMsg(double x, double y, double z)
            {
                _x = x;
                _y = y;
                _z = z;
            }

            public static string GetMessageType()
            {
                return "geometry_msgs/Vector3";
            }

            public double GetX()
            {
                return _x;
            }

            public double GetY()
            {
                return _y;
            }

            public double GetZ()
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
