using System.Collections.Generic;
using SimpleJSON;
using UnityEngine;

namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        public class RoboyPoseMsg : ROSBridgeMsg
        {

            #region PUBLIC_MEMBER_VARIABLES
            public string Name
            {
                get { return _roboyName; }
            }

            public Dictionary<string, float> XDic
            {
                get
                {
                    return _xDic;
                }
            }

            public Dictionary<string, float> YDic
            {
                get
                {
                    return _yDic;
                }
            }

            public Dictionary<string, float> ZDic
            {
                get
                {
                    return _zDic;
                }
            }

            public Dictionary<string, float> QxDic
            {
                get
                {
                    return _qxDic;
                }
            }

            public Dictionary<string, float> QyDic
            {
                get
                {
                    return _qyDic;
                }
            }

            public Dictionary<string, float> QzDic
            {
                get
                {
                    return _qzDic;
                }
            }

            public Dictionary<string, float> QwDic
            {
                get
                {
                    return _qwDic;
                }
            }

            #endregion //PUBLIC_MEMBER_VARIABLES

            #region PRIVATE_MEMBER_VARIABLES

            // TO DO MSG SO WE CAN PUBLISH IT
            private std_msgs.StringMsg _roboyNameMsg;
            private StringArrayMsg _linkNames;
            private FloatArrayMsg _xArray, _yArray, _zArray, _qxArray, _qyArray, _qzArray, _qwArray;

            private string _roboyName;
            private Dictionary<string, int> _nameIndexDic;
            private Dictionary<string, float> _xDic, _yDic, _zDic, _qxDic, _qyDic, _qzDic, _qwDic;

            #endregion //PRIVATE_MEMBER_VARIABLES

            #region PUBLIC_METHODS

            public RoboyPoseMsg(JSONNode msg)
            {
                _roboyName = msg["roboyName"];
                //Parse names to indeces so we know which name corresponds to which value
                JSONArray nameArray = msg["name"].AsArray;
                _nameIndexDic = new Dictionary<string, int>();

                for (int i = 0; i < nameArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _nameIndexDic.Add(meshName, i);
                }

                //Parse x values with the correponding name as string
                JSONArray xArray = msg["x"].AsArray;
                _xDic = CreateDictionary(nameArray, xArray);

                //Parse y values with the correponding name as string
                JSONArray yArray = msg["y"].AsArray;
                _yDic = CreateDictionary(nameArray, yArray);
                
                //Parse z values with the correponding name as string
                JSONArray zArray = msg["z"].AsArray;
                _zDic = CreateDictionary(nameArray, zArray);

                //Parse qx values with the correponding name as string
                JSONArray qxArray = msg["qx"].AsArray;
                _qxDic = CreateDictionary(nameArray, qxArray);

                //Parse qy values with the correponding name as string
                JSONArray qyArray = msg["qy"].AsArray;
                _qyDic = CreateDictionary(nameArray, qyArray);

                //Parse qz values with the correponding name as string
                JSONArray qzArray = msg["qz"].AsArray;
                _qzDic = CreateDictionary(nameArray, qzArray);

                //Parse qw values with the correponding name as string
                JSONArray qwArray = msg["qw"].AsArray;
                _qwDic = CreateDictionary(nameArray, qwArray);
            }

            // TO DO: PARSE THE DICS SO THEY ARE IN FLOAT ARRAY MSGS IN THE RIGHT ORDER
            public RoboyPoseMsg(string roboyName, List<string> linkNames, Dictionary<string, float> xDic, Dictionary<string, float> yDic, Dictionary<string, float> zDic, Dictionary<string, float> qxDic,
                Dictionary<string, float> qyDic, Dictionary<string, float> qzDic, Dictionary<string, float> qwDic)
            {
                _roboyNameMsg = new std_msgs.StringMsg("roboyName", roboyName);
                _linkNames = new StringArrayMsg("name", linkNames);

                List<float> xValues = CreateListWithLinkNames(linkNames, xDic); 
                _xArray = new FloatArrayMsg("x", xValues);

                List<float> yValues = CreateListWithLinkNames(linkNames, yDic);
                _yArray = new FloatArrayMsg("y", yValues);

                List<float> zValues = CreateListWithLinkNames(linkNames, zDic);
                _zArray = new FloatArrayMsg("z", zValues);

                List<float> qxValues = CreateListWithLinkNames(linkNames, qxDic);
                _qxArray = new FloatArrayMsg("qx", qxValues);

                List<float> qyValues = CreateListWithLinkNames(linkNames, qyDic);
                _qyArray = new FloatArrayMsg("qy", qyValues);

                List<float> qzValues = CreateListWithLinkNames(linkNames, qzDic);
                _qzArray = new FloatArrayMsg("qz", qzValues);

                List<float> qwValues = CreateListWithLinkNames(linkNames, qwDic);
                _qwArray = new FloatArrayMsg("qw", qwValues);
            }

            /// <summary>
            /// Creates pose message for one object
            /// </summary>
            /// <param name="roboyName"></param>
            /// <param name="linkName"></param>
            /// <param name="x"></param>
            /// <param name="y"></param>
            /// <param name="z"></param>
            /// <param name="qx"></param>
            /// <param name="qy"></param>
            /// <param name="qz"></param>
            /// <param name="qw"></param>
            public RoboyPoseMsg(string roboyName, string linkName,  float x, float y, float z, float qx, float qy, float qz, float qw)
            {
                _roboyNameMsg = new std_msgs.StringMsg("roboyName", roboyName);
                List<string> linkNames = new List<string>();
                linkNames.Add(linkName);
                _linkNames = new StringArrayMsg("name", linkNames);

                List<float> xValues = new List<float>();
                xValues.Add(x);
                _xArray = new FloatArrayMsg("x", xValues);

                List<float> yValues = new List<float>();
                yValues.Add(y);
                _yArray = new FloatArrayMsg("y", yValues);

                List<float> zValues = new List<float>();
                zValues.Add(z);
                _zArray = new FloatArrayMsg("z", zValues);

                List<float> qxValues = new List<float>();
                qxValues.Add(qx);
                _qxArray = new FloatArrayMsg("qx", qxValues);

                List<float> qyValues = new List<float>();
                qyValues.Add(qy);
                _qyArray = new FloatArrayMsg("qy", qyValues);

                List<float> qzValues = new List<float>();
                qzValues.Add(qz);
                _qzArray = new FloatArrayMsg("qz", qzValues);

                List<float> qwValues = new List<float>();
                qwValues.Add(qw);
                _qwArray = new FloatArrayMsg("qw", qwValues);
            }
            public static string GetMessageType()
            {
                return "common_utilities/Pose";
            }

            public override string ToString()
            {
                return "common_utilities/Pose [name =";
            }

            /// <summary>
            /// The YAML format is only needed when we publish a msg over the ROSBridge. TO DO CHANGE SO WE CAN PUBLISH IT
            /// </summary>
            /// <returns></returns>
            //public override string ToYAMLString()
            //{
            //    return "{" + _linkNames.ToYAMLString() + ", " + _xArray.ToYAMLString() + ", " + _yArray.ToYAMLString() + ", " + _zArray.ToYAMLString() +
            //        ", " + _qxArray.ToYAMLString() + ", " + _qyArray.ToYAMLString() + ", " + _qzArray.ToYAMLString() + ", " + _qwArray.ToYAMLString() +
            //        "}";
            //}

            public override string ToYAMLString()
            {
                return "{" + _roboyNameMsg.ToYAMLString() + ", " + _linkNames.ToYAMLString() + ", " + _xArray.ToYAMLString() + ", " + _yArray.ToYAMLString() + ", " + _zArray.ToYAMLString() +
                    ", " + _qxArray.ToYAMLString() + ", " + _qyArray.ToYAMLString() + ", " + _qzArray.ToYAMLString() + ", " + _qwArray.ToYAMLString() +
                    "}";
            }

            #endregion //PUBLIC_METHODS

            /// <summary>
            /// creates a List of float items according to the given dictionary and the linknames (which are values inside thr dictionary)
            /// </summary>
            /// <param name="linkNames"></param>
            /// <param name="dictionary"></param>
            /// <returns></returns>
            #region PRIVATE_METHDOS
            private List<float> CreateListWithLinkNames(List<string> linkNames, Dictionary<string, float> dictionary)
            {
                if (linkNames == null || dictionary == null)
                    return null;
                List<float> list = new List<float>();
                foreach (var linkName in linkNames)
                {
                    list.Add(dictionary[linkName]);
                }
                return list;
            }


            /// <summary>
            /// Parse given in array values(float) with the correponding name as string in the dictionary
            /// </summary>
            /// <param name="nameArray"></param>
            /// <returns></returns>
            private Dictionary<string, float> CreateDictionary(JSONArray nameArray,  JSONArray valueArray)
            {
                if (nameArray == null)
                    return null;
                Dictionary<string, float> dictionary = new Dictionary<string, float>();

                for (int i = 0; i < valueArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    dictionary.Add(meshName, valueArray[i].AsFloat);
                }
                return dictionary;
            }
            #endregion
        }
    }
}

