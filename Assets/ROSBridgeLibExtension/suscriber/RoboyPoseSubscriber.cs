using SimpleJSON;
using ROSBridgeLib.custom_msgs;
using UnityEngine;
namespace ROSBridgeLib
{
    public class RoboyPoseSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        /// <summary>
        /// Only accepts a certain amount of messages per second to prevent back-logs 
        /// Number should be close to Untiy-Frame rate
        /// </summary>
        private static int m_MaxMessagesPerSec = 80;

        /// <summary>
        /// Keeps track of time one frame ago to decide which msgs to discard
        /// </summary>
        private static float m_prevTime =0; 

        public new static string GetMessageTopic()
        {
            return "/roboy/pose";
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
            m_prevTime += Time.deltaTime;
            // Only allow max #messages per second to prevent backlog 
            if(m_prevTime> 1/(float)m_MaxMessagesPerSec)
            {
                m_prevTime = 0;
                RoboyPoseMsg pose = (RoboyPoseMsg)msg;
                RoboyManager.Instance.ReceiveMessage(pose);
            }
        }
        #endregion //PUBLIC_METHODS
    }
}