using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyPositionPublisher : ROSBridgePublisher
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/middleware/Position";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_middleware/Position";
        }

        public static string ToYAMLString(RoboyPositionMsg msg)
        {
            return msg.ToYAMLString();
        }
    }
}