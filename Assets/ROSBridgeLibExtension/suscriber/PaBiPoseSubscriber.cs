using SimpleJSON;
using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    public class PaBiPoseSubscriber : ROSBridgeSubscriber
    {
        public new static string GetMessageTopic()
        {
            return "/roboy/simulation/PaBiRoboy_demo_simplified_pose";
        }

        public new static string GetMessageType()
        {
            return "/roboy_communication_middleware/Pose";
        }

        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new RoboyPoseMsg(msg);
        }

        public new static void CallBack(ROSBridgeMsg msg)
        {
            RoboyPoseMsg pose = (RoboyPoseMsg)msg;
            RoboyManager.Instance.ReceiveMessage(pose);
        }
    }
}