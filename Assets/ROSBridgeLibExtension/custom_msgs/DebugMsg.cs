using SimpleJSON;
using System;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Warning notification 
        /// Adapt to externally defined msgs 
        ///-> https://github.com/CapChrisCap/roboy_communication/tree/feature/error-detection-msgs/roboy_communication_control/msg
        /// </summary>
        public class DebugMsg : ROSBridgeMsg
        {

            #region PRIVATE_MEMBER_VARIABLES
            /// <summary>
            /// ID of the tendon
            /// </summary>
            private string _object;

            /// <summary>
            /// Code of (error) state of object
            /// </summary>
            private int _code;

            /// <summary>
            /// Message (of state)
            /// </summary>
            private string _msg;

            /// <summary>
            /// Additional content
            /// </summary>
            private string _extra;

            /// <summary>
            /// Duration of validity
            /// </summary>
            private float _timeFrame;
            #endregion

            /// <summary>
            /// constructor 
            /// </summary>
            /// <param name="msg"></param>
            public DebugMsg(JSONNode msg)
            {
                //tendonID
                //These need to match names of externally defined msgs 
                //-> https://github.com/CapChrisCap/roboy_communication/tree/feature/error-detection-msgs/roboy_communication_control/msg
                _code = int.Parse(msg["code"]);
                _object = GazeboUtility.RemoveQuotationMarks(msg["object"].ToString()); //parse string and remove ""
                _msg = GazeboUtility.RemoveQuotationMarks(msg["msg"].ToString());
                _extra = GazeboUtility.RemoveQuotationMarks(msg["extra"].ToString());
                _timeFrame = ((float)int.Parse(msg["validityDuration"])) / 1000;
                //_timeFrame = 4f;
            }

            /// <summary>
            /// Constructor
            /// </summary>
            /// <param name="code">error / warning code == state</param>
            /// <param name="obj">string name of concerned roboy body part</param>
            /// <param name="msg">Message </param>
            /// <param name="extra">additional content</param>
            public DebugMsg(int code, string obj, string msg, string extra)
            {
                _code = code;
                _object = obj;
                _msg = msg;
                _extra = extra;
            }

            /// <summary>
            /// apparently not used since Subscriber MessageType used. 
            /// </summary>
            /// <returns></returns>
            public static string GetMessageType()
            {
                return "roboy_communication_control/WarningNotification";
            }

            /// <summary>
            /// apparently only called by developer if he wishes to -> additional debug help only
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                return "Warning for: " + _object + " , state: " + _code;
            }

            /// <summary>
            /// Only needed for send purposes -> not implemented here
            /// </summary>
            /// <returns></returns>
            public override string ToYAMLString()
            {
                throw new Exception("Not implemented yet");
            }

            /// <summary>
            /// error / info code == state
            /// </summary>
            /// <returns></returns>
            public int GetCode()
            {
                return _code;
            }

            /// <summary>
            /// Returns name of concerned roboy body part
            /// </summary>
            /// <returns></returns>
            public string GetObject()
            {
                return _object;
            }

            /// <summary>
            /// Returns Message of this notification
            /// </summary>
            /// <returns></returns>
            public string GetMessage()
            {
                return _msg;
            }

            /// <summary>
            /// returns additional information
            /// </summary>
            /// <returns></returns>
            public string GetExtra()
            {
                return _extra;
            }

            /// <summary>
            /// returns duration of validity aka. timeFrame
            /// </summary>
            /// <returns></returns>
            public float GetTimeFrame()
            {
                return _timeFrame;
            }
        }
    }
}
