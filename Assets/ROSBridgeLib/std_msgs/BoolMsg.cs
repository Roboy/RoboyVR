using System.Collections;
using System.Text;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace std_msgs
    {
        public class BoolMsg : ROSBridgeMsg
        {
            private bool _data;

            public BoolMsg(JSONNode msg)
            {
                _data = bool.Parse(msg["data"]);
            }

            public BoolMsg(bool data)
            {
                _data = data;
            }

            public static string GetMessageType()
            {
                return "std_msgs/Bool";
            }

            public bool GetData()
            {
                return _data;
            }

            public override string ToString()
            {
                return "Bool [data=" + _data + "]";
            }

            public override string ToYAMLString()
            {
                return "{\"data\" : " + _data + "}";
            }
        }
    }
}