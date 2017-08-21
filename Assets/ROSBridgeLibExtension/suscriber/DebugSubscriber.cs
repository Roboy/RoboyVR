using SimpleJSON;
using ROSBridgeLib.custom_msgs;
using System;

namespace ROSBridgeLib
{
    public class DebugSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/control/debug_notifications";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_control/DebugNotification";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new DebugMsg(msg);
        }

        /// <summary>
        /// called when new notification round -> calls core / adds to db
        /// </summary>
        /// <param name="msg"></param>
        public new static void CallBack(ROSBridgeMsg msg)
        {
            DebugMsg debug = (DebugMsg)msg;
            float timeframe = debug.GetTimeFrame();
            if (timeframe == 0) // if valid time frame
            {
                return;
            }
            if (Enum.IsDefined(typeof(DummyStates.State), debug.GetCode())) // if valid error code
            {
                Notification note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.DEBUG, (DummyStates.State)debug.GetCode(),
                    debug.GetObject(), timeframe);
                note.AddAdditionalContent(debug.GetExtra());
            }
        }

        #endregion //PUBLIC_METHODS
    }
}