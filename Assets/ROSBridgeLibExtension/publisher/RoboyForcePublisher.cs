using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class RoboyForcePublisher : ROSBridgePublisher
    {

        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        {
            return "/roboy/external_force";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_simulation/ExternalForce";
        }

        public static string ToYAMLString(ExternalForceMsg msg)
        {
            return msg.ToYAMLString();
        }

        #endregion //PUBLIC_METHODS
    }
}
