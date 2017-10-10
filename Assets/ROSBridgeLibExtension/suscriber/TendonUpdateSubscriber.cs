using ROSBridgeLib.custom_msgs;
using SimpleJSON;
using System;

namespace ROSBridgeLib
{
    public class TendonUpdateSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet -> different messages on this topic... should be alright?
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/simulation/update_tendons";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_simulation/TendonUpdate";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new TendonUpdateMsg(msg);
        }
        
        public new static void CallBack(ROSBridgeMsg msg)
        {
            TendonUpdateMsg tendon = (TendonUpdateMsg)msg;
            VRUILogic.Instance.UpdateTendon(tendon.GetTendonID(), tendon.GetForce());
        }
        #endregion //PUBLIC_METHODS
    }
}