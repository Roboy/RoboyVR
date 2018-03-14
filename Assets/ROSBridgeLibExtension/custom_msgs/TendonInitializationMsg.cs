using SimpleJSON;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Message for initialization of one tendon (defined by tendonID)
        /// Accounts for parsing errors & malformed messages 
        /// </summary>
        public class TendonInitializationMsg : ROSBridgeMsg
        {
            #region PRIVATE_MEMBER_VARIABLES
            /// <summary>
            /// ID of the tendon
            /// </summary>
            private int _tendonID;

            /// <summary>
            /// maximal force which can be applied on the tendon
            /// </summary>
            private float _maxForce; //maximal possible force applied on tendon (for comparison)

            /// <summary>
            /// list of wirepoints (3 values x,y,z in order)
            /// </summary>
            private List<Vector3> _wirepoints; //points in world space

            /// <summary>
            /// List of roboy body part names to which wirepoint is attached.
            /// this list is (supposed to be) a third of wirepoints-list size 
            /// </summary>
            private List<string> _roboyparts;
            #endregion

            /// <summary>
            /// constructor 
            /// </summary>
            /// <param name="msg"></param>
            public TendonInitializationMsg(JSONNode msg)
            {
                //tendonID
                //These need to match names of externally defined msgs 
                //-> https://github.com/Roboy/roboy_communication/tree/master/simulation/msgs
                _tendonID = int.Parse(msg["tendonID"]);
                if (!int.TryParse(msg["tendonID"], out _tendonID) || !float.TryParse(msg["maxForce"], out _maxForce))
                {
                    Debug.LogWarning("Received malformed TendonInitializationMsg: received values could not be parsed to float");
                    CreateEmptyMsg();
                    return;
                } //maxForce
                _maxForce = float.Parse(msg["maxForce"]);
                //wirepoints
                _wirepoints = new List<Vector3>();
                _roboyparts = new List<string>();
                JSONArray values = msg["wirepoints"].AsArray;
                for (int i = 0; i < values.Count; i += 3)
                {
                    Vector3 point = Vector3.zero;
                    if (!float.TryParse(values[i], out point.x) || !float.TryParse(values[i+1], out point.y) ||
                    !float.TryParse(values[i+2], out point.z))
                    {
                        Debug.LogWarning("Received malformed TendonInitializationMsg: received wirepoint values could not be parsed to float");
                        CreateEmptyMsg();
                        return;
                    }
                    _wirepoints.Add(GazeboUtility.GazeboPositionToUnity(point));
                }
                //roboyparts
                values = msg["roboyParts"].AsArray;
                for (int i = 0; i < values.Count; i++)
                {
                    _roboyparts.Add(values[i].ToString().Replace("\"", string.Empty));
                }
                if(_roboyparts.Count != _wirepoints.Count)
                {
                    Debug.LogWarning("Received malformed TendonInitializationMsg: received differing number of wirepoints and concerned body parts");
                    CreateEmptyMsg();
                    return;
                }
            }

            /// <summary>
            /// constructor 
            /// </summary>
            /// <param name="tendonID">id of tendon</param>
            /// <param name="maxForce">max force which can be applied</param>
            /// <param name="wirepoints">position of wirepoints in worldspace</param>
            /// <param name="roboyparts">string names of linked roboyparts</param>
            public TendonInitializationMsg(int tendonID, int maxForce, List<Vector3> wirepoints, List<string> roboyparts)
            {
                _tendonID = tendonID;
                _maxForce = maxForce;
                _wirepoints = wirepoints;
                _roboyparts = roboyparts;
                //throw new Exception("Not implemented yet");
            }

            /// <summary>
            /// apparently not used since Subscriber MessageType used. 
            /// </summary>
            /// <returns></returns>
            public static string GetMessageType()
            {
                return "roboy_communication_simulation/TendonInitialization";
            }

            /// <summary>
            /// apparently only called by developer if he wishes to -> additional debug help only
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                return "TendonInitializationMessage for ID: " + _tendonID;
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
            /// List of all points for this tendon
            /// </summary>
            /// <returns></returns>
            public Vector3[] GetWirepoints()
            {
                Vector3[] result = new Vector3[_wirepoints.Count];
                for (int i = 0; i < _wirepoints.Count; i++)
                {
                    result[i] = GazeboUtility.GazeboPositionToUnity(_wirepoints[i]);
                }
                return _wirepoints.ToArray();
            }

            /// <summary>
            /// Returns list of names to which each wirepoint is linked
            /// </summary>
            /// <returns></returns>
            public string[] GetRoboyParts()
            {
                return _roboyparts.ToArray();
            }

            /// <summary>
            /// Returns maximal force which can be applied on tendon
            /// </summary>
            /// <returns></returns>
            public float GetMaxForce()
            {
                return _maxForce;
            }

            #region PRIVATE_METHODS
            /// <summary>
            /// For now: create messages. in the future: maybe set some vals to -1? 
            /// </summary>
            private void CreateEmptyMsg()
            {
                _roboyparts = new List<string>();
                _wirepoints = new List<Vector3>();
                _tendonID = 0;
                _maxForce = 0;
            }
            #endregion
        }
    }
}