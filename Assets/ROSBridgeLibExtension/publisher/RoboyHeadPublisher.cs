using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyHeadPublisher : ROSBridgePublisher
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/middleware/JointCommand";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_middleware/JointCommand";
        }

        public static string ToYAMLString(RoboyPoseMsg msg)
        {
            return msg.ToYAMLString();
        }
    }
}