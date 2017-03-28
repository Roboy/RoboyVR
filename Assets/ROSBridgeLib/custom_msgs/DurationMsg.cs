using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class DurationMsg : ROSBridgeMsg
        {
            private int _duration;

            public DurationMsg(JSONNode msg)
            {
                _duration = msg["duration"].AsInt;
            }

            public DurationMsg(int duration)
            {
                _duration = duration;
            }

            public static string GetMessageType()
            {
                return "std_msgs/Int32";
            }

            public int GetDuration()
            {
                return _duration;
            }

            public override string ToString()
            {
                return "[duration=" + _duration + "]";
            }

            public override string ToYAMLString()
            {
                return "\"duration\" : " + _duration;
            }
        }
    }


}
