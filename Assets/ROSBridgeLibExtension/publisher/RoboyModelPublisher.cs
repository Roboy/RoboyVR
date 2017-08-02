using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyModelPublisher : ROSBridgePublisher
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/simulation/Model";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_simulation/Model";
        }

        public static string ToYAMLString(ModelMsg msg)
        {
            return msg.ToYAMLString();
        }
    }
}

