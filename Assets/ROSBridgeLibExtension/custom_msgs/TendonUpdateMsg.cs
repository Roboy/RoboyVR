using SimpleJSON;
using System;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Message updating force of specified tendon
        /// </summary>
        public class TendonUpdateMsg : ROSBridgeMsg
        {
            #region PRIVATE_MEMBER_VARIABLES
            /// <summary>
            /// ID of the tendon
            /// </summary>
            private int _tendonID;

            /// <summary>
            /// currently applied force
            /// </summary>
            private float _force;

            #endregion

            /// <summary>
            /// constructor 
            /// </summary>
            /// <param name="msg"></param>
            public TendonUpdateMsg(JSONNode msg)
            {
                //tendonID
                //These need to match names of externally defined msgs 
                //-> https://github.com/Roboy/roboy_communication/tree/master/simulation/msgs
                _tendonID = int.Parse(msg["tendonID"]);
                //force
                _force = float.Parse(msg["force"]);
            }

            /// <summary>
            /// Constructor
            /// </summary>
            /// <param name="tendonID">id of tendon in question</param>
            /// <param name="force">currently applied force</param>
            public TendonUpdateMsg(int tendonID, float force)
            {
                _tendonID = tendonID;
                _force = force;
            }

            /// <summary>
            /// apparently not used since Subscriber MessageType used. 
            /// </summary>
            /// <returns></returns>
            public static string GetMessageType()
            {
                return "roboy_communication_simulation/TendonUpdate";
            }

            /// <summary>
            /// apparently only called by developer if he wishes to -> additional debug help only
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                return "TendonUpdateMessage for ID: " + _tendonID + " , force: " + _force;
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
            /// ID of this tendon
            /// </summary>
            /// <returns></returns>
            public int GetTendonID()
            {
                return _tendonID;
            }

            /// <summary>
            /// Returns currently applied force
            /// </summary>
            /// <returns></returns>
            public float GetForce()
            {
                return _force;
            }
        }
    }
}
