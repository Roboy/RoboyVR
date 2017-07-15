using System.Collections;
using System.Collections.Generic;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class FloatArrayMsg : ROSBridgeMsg
        {
            private string _identifier;
            private List<float> _values;

            public FloatArrayMsg(JSONNode msg)
            {
                JSONArray values = msg[_identifier].AsArray;
                for (int i = 0; i < values.Count; i++)
                {
                    _values.Add(values[i].AsFloat);
                }
            }

            public FloatArrayMsg(string identifier, List<float> values)
            {
                _identifier = identifier;
                _values = values;
            }

            public static string GetMessageType()
            {
                return "NOT TO BE USED INDEPENDTLY";
            }

            public string GetIdentifier()
            {
                return _identifier;
            }

            public List<float> GetValues()
            {
                return _values;
            }

            public override string ToString()
            {
                string result = _identifier +  ": [";
                for (int i = 0; i < _values.Count; i++)
                {
                    result += _values[i];
                    if (i != _values.Count - 1)
                        result += ", ";
                }
                result += "]";

                return result;
            }

            // The end result should look like something like this: "qw": [0, 0, 1, 2, 1.2, 2.1]
            public override string ToYAMLString()
            {
                string result = "\"" + _identifier + "\"" + ": [";
                for (int i = 0; i < _values.Count; i++)
                {
                    result += _values[i];
                    if (i != _values.Count - 1)
                        result += ", ";
                }
                result += "]";

                return result;
            }
        }
    }
}
