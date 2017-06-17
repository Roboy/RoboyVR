using SimpleJSON;
using ROSBridgeLib.sensor_msgs;

namespace ROSBridgeLib
{
    public class RoboyCameraZedSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        {
            return "/zed/rgb/image_raw_color";
        }

        public new static string GetMessageType()
        {
            return "sensor_msgs/Image";
        }

        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            //ImageMsg from sensor messages lib
            return new ImageMsg(msg);
        }

        public new static void CallBack(ROSBridgeMsg msg)
        {
            ImageMsg image = (ImageMsg)msg;
            BeRoboyManager.Instance.ReceiveZedMessage(image);
        }

        #endregion //PUBLIC_METHODS
    }
}
