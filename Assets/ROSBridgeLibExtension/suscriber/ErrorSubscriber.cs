using SimpleJSON;
using ROSBridgeLib.custom_msgs;
using System;

namespace ROSBridgeLib
{
    public class ErrorSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/control/error_notifications";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_control/ErrorNotification";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new ErrorMsg(msg);
        }

        /// <summary>
        /// called when new notification round -> calls core / adds to db
        /// </summary>
        /// <param name="msg"></param>
        public new static void CallBack(ROSBridgeMsg msg)
        {
            ErrorMsg error = (ErrorMsg)msg;
            float timeframe = error.GetTimeFrame();
            if (timeframe == 0) // if valid time frame
            {
                return;
            }
            if (Enum.IsDefined(typeof(DummyStates.State), error.GetCode())) // if valid error code
            {
                Notification note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, (DummyStates.State)error.GetCode(),
                    error.GetObject(), timeframe);
                note.AddAdditionalContent(error.GetExtra());
            }
        }

        #endregion //PUBLIC_METHODS
    }
}