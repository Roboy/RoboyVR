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
        /// parsing errors are accounted for.
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
            /// </summary>
            private string _roboyName;

            /// <summary>
            /// Names of the concerned links of the Roboy
            /// </summary>
            private List<string> _linkNames;

            /// <summary>
            /// Positions of the concerned links 
            /// </summary>
            private Vector3[] _positions;

            /// <summary>
            /// ROtation of the concerned links
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
            /// If fail: returns empty message
            /// TODO: Are all exceptions accounted for?
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
                float[] x_values, y_values, z_values, qx_values, qy_values, qz_values, qw_values = new float[0];
                //Try parse all values to float arrays, if one not successful -> return warning, abort
                if (!ParseJsonArrayToFloats(msg["x"].AsArray, out x_values) || !ParseJsonArrayToFloats(msg["y"].AsArray, out y_values) ||
                    !ParseJsonArrayToFloats(msg["z"].AsArray, out z_values) ||
                    !ParseJsonArrayToFloats(msg["qx"].AsArray, out qx_values) || !ParseJsonArrayToFloats(msg["qy"].AsArray, out qy_values) ||
                    !ParseJsonArrayToFloats(msg["qz"].AsArray, out qz_values) || !ParseJsonArrayToFloats(msg["qw"].AsArray, out qw_values))
                {
                    Debug.LogWarning("Received Malformed RoboyPoseMsg: Some values are not floats.");
                    MakeMessageEmpty();
                    return;
                }
                else
                {
                    //if some coordinate values missing: warning & abort
                    if (x_values.Length != y_values.Length || x_values.Length != z_values.Length ||
                        x_values.Length != qw_values.Length || x_values.Length != qy_values.Length ||
                        x_values.Length != qz_values.Length || x_values.Length != qw_values.Length
                        || linkNames.Count != x_values.Length)
                    {
                        Debug.LogWarning("Received Malformed RoboyPoseMsg: Number of values in arrays does not match.");
                        MakeMessageEmpty();
                        return;
                    }
                    // save values
                    _positions = new Vector3[x_values.Length];
                    _rotations = new Quaternion[x_values.Length];

                    for (int i = 0; i < x_values.Length; i++)
                    {
                        _positions[i].x = x_values[i];
                        _positions[i].y = y_values[i];
                        _positions[i].z = z_values[i];
                        _rotations[i].x = qx_values[i];
                        _rotations[i].y = qy_values[i];
                        _rotations[i].z = qz_values[i];
                        _rotations[i].w = qw_values[i];
                    }
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
                _rotations = new Quaternion[] { GazeboUtility.UnityRotationToGazebo(rotation) };
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
            /// </summary>
            /// <param name="array">Json array containing float values</param>
            /// <param name="vals"> new array containing parsed float array</param>
            /// <returns>true if success, false otherwise</returns>
            private bool ParseJsonArrayToFloats(JSONArray array, out float[] vals)
            {
                vals = new float[array.Count];
                for (int i = 0; i < array.Count; i++)
                {
                    string value = array[i].ToString().Replace("\"", string.Empty);
                    if (!float.TryParse(value, out vals[i]))
                    {
                        return false;
                    }
                }
                return true;
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

            /// <summary>
            /// Sets all values to empty lists, strings or arrays
            /// </summary>
            private void MakeMessageEmpty()
            {
                _positions = new Vector3[0];
                _rotations = new Quaternion[0];
                _roboyName = "";
                _linkNames = new List<string>();
            }
            #endregion
        }
    }
}
