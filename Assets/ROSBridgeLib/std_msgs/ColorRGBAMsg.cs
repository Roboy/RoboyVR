using System.Collections;
using System.Text;
using SimpleJSON;


namespace ROSBridgeLib
{
    namespace std_msgs
    {
        public class ColorRGBAMsg : ROSBridgeMsg
        {
            private float _r, _g, _b, _a; //uint

            public ColorRGBAMsg(JSONNode msg)
            {
                _r = float.Parse(msg["r"]);
                _g = float.Parse(msg["g"]);
                _b = float.Parse(msg["b"]);
                _a = float.Parse(msg["a"]);
            }

            public ColorRGBAMsg(float r, float g, float b, float a)
            {
                _r = r;
                _g = g;
                _b = b;
                _a = a;
            }

            public static string GetMessageType()
            {
                return "std_msgs/ColorRGBA";
            }

            public float GetR()
            {
                return _r;
            }

            public float GetG()
            {
                return _g;
            }

            public float GetB()
            {
                return _b;
            }

            public float GetA()
            {
                return _a;
            }

            public override string ToString()
            {
                return "std_msgs/ColorRGBA [r=" + _r + ",  g=" + _g + ", b=" + _b + ", a=" + _a + "]";
            }

            public override string ToYAMLString()
            {
                return "{\"r\" : " + _r + ", \"g\": " + _g + ", \"b\": " + _b + ", \"a\": " + _a + "}";
            }
        }
    }
}
