using SimpleJSON;
using ROSBridgeLib.custom_msgs;
using System;
using UnityEngine;

namespace ROSBridgeLib
{
    public class InfoSubscriber : ROSBridgeSubscriber
    {
        #region PUBLIC_METHODS

        public new static string GetMessageTopic()
        { //no clue yet
            //throw new Exception("NO TOPIC DEFINED YET");
            return "/roboy/control/info_notifications";
        }

        public new static string GetMessageType()
        {
            return "roboy_communication_control/InfoNotification";
        }

        /// <summary>
        /// Parses received msg to intermediate TendonInitializationMsg
        /// </summary>
        /// <param name="msg">Message received via ROSBridge</param>
        /// <returns></returns>
        public new static ROSBridgeMsg ParseMessage(JSONNode msg)
        {
            return new InfoMsg(msg);
        }

        /// <summary>
        /// called when new notification round -> calls core / adds to db
        /// </summary>
        /// <param name="msg"></param>
        public new static void CallBack(ROSBridgeMsg msg)
        {
            Debug.Log("Found INFO Msg.");
            InfoMsg info = (InfoMsg)msg;
            float timeframe = info.GetTimeFrame();
            if (timeframe == 0) // if valid time frame
            {
                return;
            }
            if (Enum.IsDefined(typeof(DummyStates.State), info.GetCode())) // if valid error code
            {
                Notification note = VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.INFO, (DummyStates.State)info.GetCode(),
                    info.GetObject(), timeframe);
                note.AddAdditionalContent(info.GetExtra());
            }
        }

        #endregion //PUBLIC_METHODS
    }
}