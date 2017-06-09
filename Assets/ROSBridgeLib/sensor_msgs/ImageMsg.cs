using System.Collections;
using System.Text;
using SimpleJSON;
using ROSBridgeLib.std_msgs;
using UnityEngine;

/**
 * Define a compressed image message. Note: the image is assumed to be in Base64 format.
 * Which seems to be what is normally found in json strings. Documentation. Got to love it.
 * 
 * @author Michael Jenkin, Robert Codd-Downey and Andrew Speers
 * @version 3.1
 */

namespace ROSBridgeLib
{
    namespace sensor_msgs
    {
        public class ImageMsg : ROSBridgeMsg
        {
            
            private byte[] _data;
            private HeaderMsg _header;


            public ImageMsg(JSONNode msg)
            {
                
                _header = new HeaderMsg(msg["header"]);
                _data = System.Convert.FromBase64String(msg[4]);
            }

            public ImageMsg(HeaderMsg header, byte[] data)
            {
                _header = header;               
                _data = data;
            }

            public byte[] GetImage()
            {
                return _data;
            }

            public static string GetMessageType()
            {
                return "sensor_msgs/Image";
            }

            public override string ToString()
            {
                return "Image [size=" + _data.Length + ", Header " + _header.ToString() + "]";
            }

            public override string ToYAMLString()
            {
                return "{\"data\" : \"" + System.Convert.ToBase64String(_data) + "\", \"header\" : " + _header.ToYAMLString() + "}";
            }
        }
    }
}
