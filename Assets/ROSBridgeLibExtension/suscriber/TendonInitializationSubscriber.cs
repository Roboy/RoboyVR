using SimpleJSON;
using ROSBridgeLib.custom_msgs;

namespace ROSBridgeLib
{
    /// <summary>
    /// subscriber listening for new tendons. parses and adds these to UIdatabase
    /// </summary>
    public class TendonInitializationSubscriber : ROSBridgeSubscriber
    {

        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/simulation/init_tendons";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_simulation/TendonInitialization";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new TendonInitializationMsg(msg);
        }


        public new static void CallBack(ROSBridgeMsg msg)
        {
            TendonInitializationMsg tendon = (TendonInitializationMsg)msg;
            if (tendon != null)
                VRUILogic.Instance.AddTendon(tendon.GetTendonID(), tendon.GetWirepoints(),
                    tendon.GetRoboyParts(), tendon.GetMaxForce());
        }

        #endregion //PUBLIC_METHODS
    }
}