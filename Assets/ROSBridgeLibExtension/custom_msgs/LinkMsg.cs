using System.Collections;
using System.Collections.Generic;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class LinkMsg : ROSBridgeMsg
        {
            private string _data;

            public LinkMsg(JSONNode msg)
            {
                _data = msg["data"];
            }

            public LinkMsg(string data)
            {
                _data = data;
            }

            public static string GetMessageType()
            {
                return "std_msgs/String";
            }

            public string GetData()
            {
                return _data;
            }

            public override string ToString()
            {
                return "String [data=" + _data + "]";
            }

            public override string ToYAMLString()
            {
                return "\"name\" : \"" + _data + "\"";
            }
        }
    }
}
