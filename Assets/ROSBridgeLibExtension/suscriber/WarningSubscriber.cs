using SimpleJSON;
using ROSBridgeLib.custom_msgs;
using System;
using UnityEngine;

namespace ROSBridgeLib
{
    public class WarningSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/control/warning_notifications";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_control/WarningNotification";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new WarningMsg(msg);
        }

        /// <summary>
        /// called when new notification round -> calls core / adds to db
        /// </summary>
        /// <param name="msg"></param>
        public new static void CallBack(ROSBridgeMsg msg)
        {
            Debug.Log("Warning MSg received");
            WarningMsg warning = (WarningMsg)msg;
            float timeframe = warning.GetTimeFrame();
            if (timeframe == 0) // if valid time frame
            {
                return;
            }
            if (Enum.IsDefined(typeof(DummyStates.State), warning.GetCode())) // if valid error code
            {
                Debug.Log("Adding ...");

                Debug.Log("object: " + warning.GetObject());
                Notification note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, (DummyStates.State)warning.GetCode(),
                    warning.GetObject(), timeframe);
                note.AddAdditionalContent(warning.GetExtra());
            }
        }

        #endregion //PUBLIC_METHODS
    }
}