using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Parses received message to an int value _duration. does not return error if parsing fails, instead value set to 0.
        /// </summary>
        public class DurationMsg : ROSBridgeMsg
        {
            private int _duration;

            public DurationMsg(JSONNode msg)
            {
                _duration = msg["duration"].AsInt; // no error
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
