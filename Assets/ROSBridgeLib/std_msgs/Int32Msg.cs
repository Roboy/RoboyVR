using System.Collections;
using System.Text;
using SimpleJSON;

namespace ROSBridgeLib {
	namespace std_msgs {
		public class Int32Msg : ROSBridgeMsg {
			private int _data;
            private string _identifier;
			
			public Int32Msg(JSONNode msg) {
				_data = msg["data"].AsInt;
			}
			
			public Int32Msg(int data) {
				_data = data;
			}

            public Int32Msg(string identifier, int data)
            {
                _data = data;
                _identifier = identifier;
            }
			
			public static string GetMessageType() {
				return "std_msgs/Int32";
			}
			
			public int GetData() {
				return _data;
			}
			
			public override string ToString() {
				return "Int32 [data=" + _data + "]";
			}
			
            /// <summary>
            /// If the identifier is not set then we expect the message to be sent alone with "data" as standard identifier, othewise we expect
            /// this message to be sent in another message therefore without opening and closing brackets.
            /// </summary>
            /// <returns></returns>
			public override string ToYAMLString() {
                if (!string.IsNullOrEmpty(_identifier))
                    return "\"" + _identifier + "\" : " + _data;
                else
				    return "{\"data\" : " + _data + "}";
			}
		}
	}


}