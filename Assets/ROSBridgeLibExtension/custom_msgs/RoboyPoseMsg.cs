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
                _xDic = new Dictionary<string, float>();

                for (int i = 0; i < xArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _xDic.Add(meshName, xArray[i].AsFloat);
                }

                //Parse y values with the correponding name as string
                JSONArray yArray = msg["y"].AsArray;
                _yDic = new Dictionary<string, float>();

                for (int i = 0; i < yArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _yDic.Add(meshName, yArray[i].AsFloat);
                }

                //Parse z values with the correponding name as string
                JSONArray zArray = msg["z"].AsArray;
                _zDic = new Dictionary<string, float>();

                for (int i = 0; i < zArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _zDic.Add(meshName, zArray[i].AsFloat);
                }

                //Parse qx values with the correponding name as string
                JSONArray qxArray = msg["qx"].AsArray;
                _qxDic = new Dictionary<string, float>();

                for (int i = 0; i < qxArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _qxDic.Add(meshName, qxArray[i].AsFloat);
                }

                //Parse qy values with the correponding name as string
                JSONArray qyArray = msg["qy"].AsArray;
                _qyDic = new Dictionary<string, float>();

                for (int i = 0; i < qyArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _qyDic.Add(meshName, qyArray[i].AsFloat);
                }

                //Parse qz values with the correponding name as string
                JSONArray qzArray = msg["qz"].AsArray;
                _qzDic = new Dictionary<string, float>();

                for (int i = 0; i < qzArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _qzDic.Add(meshName, qzArray[i].AsFloat);
                }

                //Parse qw values with the correponding name as string
                JSONArray qwArray = msg["qw"].AsArray;
                _qwDic = new Dictionary<string, float>();

                for (int i = 0; i < qwArray.Count; i++)
                {
                    string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
                    _qwDic.Add(meshName, qwArray[i].AsFloat);
                }
            }

            // TO DO: PARSE THE DICS SO THEY ARE IN FLOAT ARRAY MSGS IN THE RIGHT ORDER
            public RoboyPoseMsg(string roboyName, List<string> linkNames, Dictionary<string, float> xDic, Dictionary<string, float> yDic, Dictionary<string, float> zDic, Dictionary<string, float> qxDic,
                Dictionary<string, float> qyDic, Dictionary<string, float> qzDic, Dictionary<string, float> qwDic)
            {
                _roboyNameMsg = new std_msgs.StringMsg("roboyName", roboyName);
                _linkNames = new StringArrayMsg("name", linkNames);

                List<float> xValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    xValues.Add(xDic[linkName]);
                }
                _xArray = new FloatArrayMsg("x", xValues);

                List<float> yValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    yValues.Add(yDic[linkName]);
                }
                _yArray = new FloatArrayMsg("y", yValues);

                List<float> zValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    zValues.Add(zDic[linkName]);
                }
                _zArray = new FloatArrayMsg("z", zValues);

                List<float> qxValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    qxValues.Add(qxDic[linkName]);
                }
                _qxArray = new FloatArrayMsg("qx", qxValues);

                List<float> qyValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    qyValues.Add(qyDic[linkName]);
                }
                _qyArray = new FloatArrayMsg("qy", qyValues);

                List<float> qzValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    qzValues.Add(qzDic[linkName]);
                }
                _qzArray = new FloatArrayMsg("qz", qzValues);

                List<float> qwValues = new List<float>();
                foreach (var linkName in linkNames)
                {
                    qwValues.Add(qwDic[linkName]);
                }
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
        }
    }
}

