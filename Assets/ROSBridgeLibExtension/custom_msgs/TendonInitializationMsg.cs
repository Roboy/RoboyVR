using SimpleJSON;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Message for initialization of one tendon
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
                //maxForce
                _maxForce = float.Parse(msg["maxForce"]);
                //wirepoints
                _wirepoints = new List<Vector3>();
                _roboyparts = new List<string>();
                JSONArray values = msg["wirepoints"].AsArray;
                for (int i = 0; i < values.Count; i += 3)
                {
                    _wirepoints.Add(new Vector3(float.Parse(values[i]), float.Parse(values[i+1]), float.Parse(values[i+2])));
                }
                //roboyparts
                values = msg["roboyParts"].AsArray;
                for (int i = 0; i < values.Count; i++)
                {
                    _roboyparts.Add(values[i].ToString().Replace("\"", string.Empty));
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
        }
    }
}