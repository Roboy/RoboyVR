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
        public class RoboyPositionMsg : ROSBridgeMsg
        {
            public double x
            {
                get
                {
                    return _x;
                }
            }

            public double y
            {
                get
                {
                    return _y;
                }
            }


            public double z
            {
                get
                {
                    return _z;
                }
            }

            private double _x;
            private double _y;
            private double _z;
            

            public RoboyPositionMsg(JSONNode msg)
            {
            //TODO implement in the future
            }

            public RoboyPositionMsg(double x, double y, double z)
            {
                _x = x;
                _y = y;
                _z = z;
                
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
