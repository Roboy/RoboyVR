using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyHandsPublisher : ROSBridgePublisher
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/simulation/hands/external_pose";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_middleware/Pose";
        }

        public static string ToYAMLString(RoboyPoseMsg msg)
        {
            return msg.ToYAMLString();
        }
    }
}

