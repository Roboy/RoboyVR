using System.Collections;
using System.Text;
using SimpleJSON;



namespace ROSBridgeLib {
	namespace turtlesim {
        /**
         * Define a turtle pose message. This has been hand-crafted from the corresponding
         * turtle message file.
         * 
         * Version History
         * 3.1 - changed methods to start with an upper case letter to be more consistent with c#
         * style.
         * 3.0 - modification from hand crafted version 2.0
         * 
         */
        public class PoseMsg : ROSBridgeMsg {
			private float _x, _y, _theta, _linear_velocity, _angular_velocity;

            /// <summary>
            /// This constructor is called when you receive a message from the ROSBridge.
            /// </summary>
            /// <param name="msg"></param>
			public PoseMsg(JSONNode msg) {
				_x = float.Parse(msg["x"]);
				_y = float.Parse (msg["y"]);
				_theta = float.Parse (msg["theta"]);
				_linear_velocity = float.Parse(msg["linear_velocity"]);
				_angular_velocity = float.Parse (msg["angular_velocity"]);
			}

            /// <summary>
            /// This constuctor can be used to construct a message in Unity and send it over the ROSBridge.
            /// </summary>
            /// <param name="x"></param>
            /// <param name="y"></param>
            /// <param name="theta"></param>
            /// <param name="linear_velocity"></param>
            /// <param name="angular_velocity"></param>
			public PoseMsg(float x, float y, float theta, float linear_velocity, float angular_velocity) {
				_x = x;
				_y = y;
				_theta = theta;
				_linear_velocity = linear_velocity;
				_angular_velocity = angular_velocity;
			}
			
            /// <summary>
            /// This is called when you send the message over the ROSBridge. It must be equal to the type of the input of the receiving node.
            /// </summary>
            /// <returns></returns>
			public static string GetMessageType() {
				return "turtlesim/Pose";
			}
			
			public float GetX() {
				return _x;
			}
			
			public float GetY() {
				return _y;
			}
			
			public float GetTheta() {
				return _theta;
			}

			public float GetLinear_Velocity() {
				return _linear_velocity;
			}
			
			public float GetAngular_Velocity() {
				return _angular_velocity;
			}
			
			public override string ToString() {
				return "turtlesim/Pose [x=" + _x + ",  y=" + _y + ", theta=" + _theta +  
					", linear_velocity=" + _linear_velocity + ", angular_velocity=" + _angular_velocity + "]";
			}
			

			/// <summary>
            /// You need this function to send a message over the ROSBridge to the desired ROS node as YAML is the standard format for this.
            /// </summary>
            /// <returns></returns>
			public override string ToYAMLString() {
				return "{\"x\": " + _x + ", \"y\": " + _y + ", \"theta\": " + _theta + 
					", \"linear_velocity\": " + _linear_velocity + ", \"angular_velocity\": " + _angular_velocity +"}";
			}
		}
	}
}
