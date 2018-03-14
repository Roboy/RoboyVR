using System.Collections.Generic;
using ROSBridgeLib.std_msgs;
using SimpleJSON;
using UnityEngine;


namespace ROSBridgeLib
{
    namespace custom_msgs
    {
        /// <summary>
        /// Msg containing info about command (insert / remove) and type of concerned object (model/world)
        /// NOT REVIEWED
        /// </summary>
        public class ModelMsg : ROSBridgeMsg
        {
            /// <summary>
            /// 0 stands for Removal of models
            /// 1 stands for Insertions of models
            /// </summary>
            public Int32Msg Operation
            {
                get
                {
                    return _Operation;
                }
            }

            /// <summary>
            /// 0 stands for World insertion/removal
            /// 1 stands for Model insertion/removal
            /// </summary>
            public Int32Msg Type
            {
                get
                {
                    return _Type;
                }
            }

            /// <summary>
            /// Objects which you want to remove/insert
            /// </summary>
            public StringArrayMsg Objects
            {
                get
                {
                    return _Objects;
                }
            }

            /// <summary>
            /// The corresponding positions
            /// </summary>
            public FloatArrayMsg Positions
            {
                get
                {
                    return _Positions;
                }
            }

            private Int32Msg _Operation;
            private Int32Msg _Type;
            private StringArrayMsg _Objects;
            private FloatArrayMsg _Positions;


            public ModelMsg(JSONNode msg)
            {
                throw new System.NotImplementedException();
            }

            public ModelMsg(int operation, int type, List<string> objects, List<Vector3> positions)
            {
                _Operation = new Int32Msg("operation", operation);
                _Type = new Int32Msg("type", type);
                _Objects = new StringArrayMsg("objects", objects);
                List<float> values = new List<float>();
                foreach (var pos in positions)
                {
                    values.Add(pos.x);
                    values.Add(pos.y);
                    values.Add(pos.z);
                }
                _Positions = new FloatArrayMsg("positions", values);

            }

            public static string GetMessageType()
            {
                return "roboy_communication_simulation/Model";
            }

            public override string ToString()
            {
                return "roboy_communication_simulation/model [name=";
            }

            public override string ToYAMLString()
            {
                //return string.Format("{{{0}, {1}, {2}, {3}}}", _Operation.ToYAMLString(), _Type.ToYAMLString(), _Objects.ToYAMLString(), _Positions.ToYAMLString());
                return "{" + _Operation.ToYAMLString() + ", " + _Type.ToYAMLString() + ", " + _Objects.ToYAMLString() + ", " + _Positions.ToYAMLString() + "}";
            }
        }
    }
}
