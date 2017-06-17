using SimpleJSON;
using ROSBridgeLib.sensor_msgs;

namespace ROSBridgeLib
{
    public class RoboyCameraSimSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        {
            return "/roboy/camera/image_raw";         
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
            BeRoboyManager.Instance.ReceiveSimMessage(image);
        }

        #endregion //PUBLIC_METHODS
    }
}
