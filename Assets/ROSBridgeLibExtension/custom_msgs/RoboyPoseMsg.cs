using System.Collections.Generic;
using SimpleJSON;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// RoboyPoseMessage containing all links and their respective positions and rotations. 
        /// Incoming messages are parsed into Unity coordinate space, outgoing to Gazebo coordinate space. 
        /// </summary>
        public class RoboyPoseMsg : ROSBridgeMsg
        {

            #region PUBLIC_MEMBER_VARIABLES
            public string Name
            {
                get
                {
                    return _roboyName;
                }
            }
            
            public List<string> linkNames
            {
                get
                {
                    return _linkNames;
                }
            }

            public Vector3[] positions
            {
                get
                {
                    return _positions;
                }
            }

            public Quaternion[] rotations
            {
                get
                {
                    return _rotations;
                }
            }
            #endregion //PUBLIC_MEMBER_VARIABLES

            #region PRIVATE_MEMBER_VARIABLES

            /// <summary>
            /// Name of the concerned Roboy
            /// THIS DOES NOT OFFICIALLY EXIST IN THE GIVEN MESSAGE TYPE YET
            /// </summary>
            private string _roboyName;

            /// <summary>
            /// Names of the concerned links of the Roboy
            /// </summary>
            private List<string> _linkNames;

            /// <summary>
            /// Positions of the concerned links 
            /// IMPORTANT: ORDER MUS BE EQUIVALENT TO LINK NAMES LIST
            /// </summary>
            private Vector3[] _positions;

            /// <summary>
            /// ROtation of the concerned links
            /// IMPORTANT: ORDER MUS BE EQUIVALENT TO LINK NAMES LIST
            /// </summary>
            private Quaternion[] _rotations;

            /// <summary>
            /// if set to true, the positions and rotations will be in GAZEBO COORD system
            /// </summary>
            private bool _outgoing;
            #endregion //PRIVATE_MEMBER_VARIABLES

            #region PUBLIC_METHODS

            /// <summary>
            /// Parse given message to RoboyPoseMsg type
            /// FIXME: If malformed message, excepions might occure here!!!
            /// </summary>
            /// <param name="msg"></param>
            public RoboyPoseMsg(JSONNode msg)
            {
                //Initialize everything
                _linkNames = new List<string>();

                //Is not defined in the official message type yet
                _roboyName = msg["roboyName"];

                JSONArray nameArray = msg["name"].AsArray;

                for (int i = 0; i < nameArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _linkNames.Add(meshName);
                }

                // x positions 
                float[] vals = ParseJsonArrayToFloats(msg["x"].AsArray);
                _positions = new Vector3[vals.Length];
                for (int i = 0; i < vals.Length; i++)
                {
                    _positions[i].x = vals[i];
                }

                // y position
                vals = ParseJsonArrayToFloats(msg["y"].AsArray);
                for (int i = 0; i < vals.Length; i++)
                {
                    _positions[i].y = vals[i];
                }
                // z position
                vals = ParseJsonArrayToFloats(msg["z"].AsArray);
                for (int i = 0; i < vals.Length; i++)
                {
                    _positions[i].z = vals[i];
                }

                // rotations 
                // x rotation
                vals = ParseJsonArrayToFloats(msg["qx"].AsArray);
                _rotations = new Quaternion[vals.Length];
                for (int i = 0; i < vals.Length; i++)
                {
                    _rotations[i].x = vals[i];
                }
                // y rotation
                vals = ParseJsonArrayToFloats(msg["qy"].AsArray);
                for (int i = 0; i < vals.Length; i++)
                {
                    _rotations[i].y = vals[i];
                }
                // z rotation
                vals = ParseJsonArrayToFloats(msg["qz"].AsArray);
                for (int i = 0; i < vals.Length; i++)
                {
                    _rotations[i].z = vals[i];

                }
                // w value
                vals = ParseJsonArrayToFloats(msg["qw"].AsArray);
                for (int i = 0; i < vals.Length; i++)
                {
                    _rotations[i].w = vals[i];

                }
                // Transform from gazebo coordinate system to unity coordinate system
                _outgoing = false;
                for (int i = 0; i < _positions.Length; i++)
                {
                    _positions[i] = GazeboUtility.GazeboPositionToUnity(_positions[i]);
                    _rotations[i] = GazeboUtility.GazeboRotationToUnity(_rotations[i]);
                }
            }

            /// <summary>
            /// Create message based on given params. Attention: link names, positions and rotation need to correspond to same index (*obviously*)
            /// FIXME: No security checks here
            /// </summary>
            /// <param name="roboyName">Name of roboy - TODO for now: not of interest / use</param>
            /// <param name="linkNames">Name of the links to be updated</param>
            /// <param name="positions">position of these links in UNITY COORDINATE SPACE</param>
            /// <param name="rotation">rotation of these links in UNITY COORDINATE SPACE</param>
            public RoboyPoseMsg(string roboyName, List<string> linkNames, Vector3[] positions, Quaternion[] rotations)
            {
                // translation from unity to gazebo coordinate system
                _positions = new Vector3[positions.Length];
                _rotations = new Quaternion[rotations.Length];
                _outgoing = true;
                for (int i = 0; i < positions.Length; i++)
                {
                    _positions[i] = GazeboUtility.UnityPositionToGazebo(positions[i]);
                    _rotations[i] = GazeboUtility.UnityRotationToGazebo(rotations[i]);
                }
                _roboyName = roboyName;
                _linkNames = linkNames;
            }


            /// <summary>
            /// Creates pose message for one link
            /// MAINLY FOR TEST PURPOSES
            /// </summary>
            /// <param name="roboyName">Name of the roboy: TODO - for now not of interest</param>
            /// <param name="linkName">the concerned link</param>
            /// <param name="position">position of the link in UNITY COORD SPACE</param>
            /// <param name="rotation">rotation of the link in UNITY COORD SPACE</param>
            public RoboyPoseMsg(string roboyName, string linkName, Vector3 position, Quaternion rotation)
            {
                //_roboyNameMsg = new std_msgs.StringMsg("roboyName", roboyName);
                _linkNames = new List<string>();
                _linkNames.Add(linkName);
                _positions = new Vector3[] { GazeboUtility.UnityPositionToGazebo(position) };
                _rotations = new Quaternion[] { GazeboUtility.UnityRotationToGazebo(rotation)};
            }

            public static string GetMessageType()
            {
                return "common_utilities/Pose";
            }

            /// <summary>
            /// Returns whether the given values are in Unity coordinate space (true)
            /// </summary>
            /// <returns></returns>
            public bool IsInUnityCoordinateSpace()
            {
                return !_outgoing;
            }

            /// <summary>
            /// NOT IMPLEMENTED (NEEDED) Use ToYAMLString
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                throw new System.NotImplementedException();
            }

            /// <summary>
            /// The YAML format is only needed when we publish a msg over the ROSBridge.
            /// </summary>
            /// <returns></returns>
            public override string ToYAMLString()
            {
                string result = "{\"name\":[";
                //link names
                foreach (string name in _linkNames)
                {
                    result += "\"" + name + "\",";
                }
                result = result.Remove(result.Length - 1) + "]";
              
                result += ", \"x\":" + ParseFLoatArrayToString(GetVector3ArrayPartly(_positions, 0));
                result += ", \"y\":" + ParseFLoatArrayToString(GetVector3ArrayPartly(_positions, 1));
                result += ", \"z\":" + ParseFLoatArrayToString(GetVector3ArrayPartly(_positions, 2));
                result += ", \"qx\":" + ParseFLoatArrayToString(GetQuaternionArrayPartly(_rotations, 0));
                result += ", \"qy\":" + ParseFLoatArrayToString(GetQuaternionArrayPartly(_rotations, 1));
                result += ", \"qz\":" + ParseFLoatArrayToString(GetQuaternionArrayPartly(_rotations, 2));
                result += ", \"qw\":" + ParseFLoatArrayToString(GetQuaternionArrayPartly(_rotations, 3));
                result += "}";
                return result;
            }

            #endregion //PUBLIC_METHODS

            #region PRIVATE_METHDOS            
            /// <summary>
            /// parses array to float values and returns these in form of an array. 
            /// All " are removed. 
            /// FIXME: Malformed messages cause this part to crash when parsing fails (but faster this way)
            /// </summary>
            /// <param name="array"></param>
            /// <returns></returns>
            private float[] ParseJsonArrayToFloats(JSONArray array)
            {
                float[] vals = new float[array.Count];
                for (int i = 0; i < array.Count; i++)
                {
                    string value = array[i].ToString().Replace("\"", string.Empty);
                    vals[i] = float.Parse(value);
                }
                return vals;
            }

            /// <summary>
            /// parses float values to string whith json array format.  -> [0.0, 0.0, 0.0]
            /// " are added. 
            /// </summary>
            /// <param name="array"></param>
            /// <returns></returns>
            private string ParseFLoatArrayToString(float[] array)
            {
                string result = "[";

                foreach (float val in array)
                {
                    result += val.ToString() + ",";
                }
                return result.Remove(result.Length - 1) + "]";
            }

            /// <summary>
            /// returns float array consistion of the "pos"{0;1;2} value of the entry of all entries of the array
            /// NO SECURITY CHECKS
            /// </summary>
            /// <param name="array"></param>
            /// <param name="pos"></param>
            /// <returns></returns>
            private float[] GetVector3ArrayPartly(Vector3[] array, int pos)
            {
                float[] result = new float[array.Length];
                for (int i = 0; i < array.Length; i++)
                {

                    result[i] = array[i][pos];
                }
                return result;
            }

            /// <summary>
            /// returns float array consistion of the "pos"{0;1;2;3} value of the entry of all entries of the array
            /// NO SECURITY CHECKS
            /// </summary>
            /// <param name="array"></param>
            /// <param name="pos"></param>
            /// <returns></returns>
            private float[] GetQuaternionArrayPartly(Quaternion[] array, int pos)
            {
                float[] result = new float[array.Length];
                for (int i = 0; i < array.Length; i++)
                {

                    result[i] = array[i][pos];
                }
                return result;
            }
            #endregion
        }
    }
}
