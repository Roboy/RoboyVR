using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyHeadPublisher : ROSBridgePublisher
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/middleware/JointCommandRevolute";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_middleware/JointCommandRevolute";
        }

        public static string ToYAMLString(RoboyPoseMsg msg)
        {
            return msg.ToYAMLString();
        }
    }
}