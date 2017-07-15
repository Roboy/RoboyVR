using System.Collections;
using System.Collections.Generic;
using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class LinkArrayMsg : ROSBridgeMsg
        {
            private List<string> _linkNames;

            public LinkArrayMsg(JSONNode msg)
            {
                JSONArray linkNames = msg["name"].AsArray;
                foreach (var linkName in linkNames)
                {
                    _linkNames.Add(linkName.ToString());
                }
            }

            public LinkArrayMsg(List<string> linkNames)
            {
                _linkNames = linkNames;
            }

            public static string GetMessageType()
            {
                return "NOT TO BE USED INDEPENDTLY";
            }

            public List<string> GetLinkNames()
            {
                return _linkNames;
            }

            public override string ToString()
            {
                string result = "linknames: [";
                for (int i = 0; i < _linkNames.Count; i++)
                {
                    result += "\"" + _linkNames[i] + "\"";
                    if (i != _linkNames.Count - 1)
                        result += ", ";
                }
                result += "]";

                return result;
            }

            public override string ToYAMLString()
            {
                string result = "\"name\": [";
                for (int i = 0; i < _linkNames.Count; i++)
                {
                    // add each link name in quotation marks so it looks like : "hip", "thigh_left" ..
                    result += "\"" + _linkNames[i] + "\"";
                    // add a comma to each link until last one
                    if (i != _linkNames.Count - 1)
                        result += ", ";
                }
                result += "]";
                // end result should look like this : "name": ["hip", "thigh_left", ...]
                return result;
            }
        }
    }
}
