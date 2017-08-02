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
        public class ModelMsg : ROSBridgeMsg
        {
            public std_msgs.StringMsg Operation
            {
                get
                {
                    return _Operation;
                }
            }

            public std_msgs.StringMsg Type
            {
                get
                {
                    return _Type;
                }
            }


            public custom_msgs.StringArrayMsg Objects
            {
                get
                {
                    return _Objects;
                }
            }

            private std_msgs.StringMsg _Operation;
            private std_msgs.StringMsg _Type;
            private custom_msgs.StringArrayMsg _Objects;
            

            public ModelMsg(JSONNode msg)
            {
            //TODO implement in the future
            }

            public ModelMsg(string operation, string type, List<string> objects)
            {
                _Operation = new StringMsg(operation);
                _Type = new StringMsg(type);
                _Objects = new StringArrayMsg("objects", objects);
                
            }

            public static string GetMessageType()
            {
                return "roboy_communication_simulation/Model";
            }

            public override string ToString()
            {
                return "roboy_communication_simulation/Model [name=";
            }

            public override string ToYAMLString()
            {
                return "{" + _Operation.ToYAMLString() + ", " + _Type.ToYAMLString() + ", " + _Objects.ToYAMLString() + "}";
            }
        }
    }
}
