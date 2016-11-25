﻿using System.Collections.Generic;
using SimpleJSON;


public class RoboyPoseMsg : ROSBridgeMsg
{

    #region PUBLIC_MEMBER_VARIABLES
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

    public Dictionary<string, float> RollDic
    {
        get
        {
            return _rollDic;
        }
    }

    public Dictionary<string, float> PitchDic
    {
        get
        {
            return _pitchDic;
        }
    }

    public Dictionary<string, float> YawDic
    {
        get
        {
            return _yawDic;
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

    public Dictionary<string, float> V00Dic
    {
        get
        {
            return _v00Dic;
        }
    }

    public Dictionary<string, float> V01Dic
    {
        get
        {
            return _v01Dic;
        }
    }

    public Dictionary<string, float> V02Dic
    {
        get
        {
            return _v02Dic;
        }
    }

    public Dictionary<string, float> V03Dic
    {
        get
        {
            return _v03Dic;
        }
    }

    public Dictionary<string, float> V10Dic
    {
        get
        {
            return _v10Dic;
        }
    }

    public Dictionary<string, float> V11Dic
    {
        get
        {
            return _v11Dic;
        }
    }

    public Dictionary<string, float> V12Dic
    {
        get
        {
            return _v12Dic;
        }
    }

    public Dictionary<string, float> V13Dic
    {
        get
        {
            return _v13Dic;
        }
    }

    public Dictionary<string, float> V20Dic
    {
        get
        {
            return _v20Dic;
        }
    }

    public Dictionary<string, float> V21Dic
    {
        get
        {
            return _v21Dic;
        }
    }

    public Dictionary<string, float> V22Dic
    {
        get
        {
            return _v22Dic;
        }
    }

    public Dictionary<string, float> V23Dic
    {
        get
        {
            return _v23Dic;
        }
    }

    public Dictionary<string, float> V30Dic
    {
        get
        {
            return _v30Dic;
        }
    }

    public Dictionary<string, float> V31Dic
    {
        get
        {
            return _v31Dic;
        }
    }

    public Dictionary<string, float> V32Dic
    {
        get
        {
            return _v32Dic;
        }
    }

    public Dictionary<string, float> V33Dic
    {
        get
        {
            return _v33Dic;
        }
    }




    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES

    private Dictionary<string, int> _nameIndexDic;
    private Dictionary<string, float> _xDic, _yDic, _zDic, _rollDic, _pitchDic, _yawDic, _qxDic, _qyDic, _qzDic, _qwDic;

    private Dictionary<string, float> 
        _v00Dic,
        _v01Dic,
        _v02Dic,
        _v03Dic,
        _v10Dic,
        _v11Dic,
        _v12Dic,
        _v13Dic,
        _v20Dic,
        _v21Dic,
        _v22Dic,
        _v23Dic,
        _v30Dic,
        _v31Dic,
        _v32Dic,
        _v33Dic;

    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public RoboyPoseMsg(JSONNode msg)
    {
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

        //Parse roll values with the correponding name as string
        JSONArray rollArray = msg["roll"].AsArray;
        _rollDic = new Dictionary<string, float>();

        for (int i = 0; i < rollArray.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _rollDic.Add(meshName, rollArray[i].AsFloat);
        }

        //Parse pitch values with the correponding name as string
        JSONArray pitchArray = msg["pitch"].AsArray;
        _pitchDic = new Dictionary<string, float>();

        for (int i = 0; i < pitchArray.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _pitchDic.Add(meshName, pitchArray[i].AsFloat);
        }

        //Parse yaw values with the correponding name as string
        JSONArray yawArray = msg["yaw"].AsArray;
        _yawDic = new Dictionary<string, float>();

        for (int i = 0; i < yawArray.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _yawDic.Add(meshName, yawArray[i].AsFloat);
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

        //Parse v00 of the rotation matrix values with the correponding name as string
        JSONArray v00Array = msg["v00"].AsArray;
        _v00Dic = new Dictionary<string, float>();

        for (int i = 0; i < v00Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v00Dic.Add(meshName, v00Array[i].AsFloat);
        }

        //Parse v01 of the rotation matrix values with the correponding name as string
        JSONArray v01Array = msg["v01"].AsArray;
        _v01Dic = new Dictionary<string, float>();

        for (int i = 0; i < v01Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v01Dic.Add(meshName, v01Array[i].AsFloat);
        }

        //Parse v02 of the rotation matrix values with the correponding name as string
        JSONArray v02Array = msg["v02"].AsArray;
        _v02Dic = new Dictionary<string, float>();

        for (int i = 0; i < v02Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v02Dic.Add(meshName, v02Array[i].AsFloat);
        }

        //Parse v03 of the rotation matrix values with the correponding name as string
        JSONArray v03Array = msg["v03"].AsArray;
        _v03Dic = new Dictionary<string, float>();

        for (int i = 0; i < v03Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v03Dic.Add(meshName, v03Array[i].AsFloat);
        }

        //Parse v10 of the rotation matrix values with the correponding name as string
        JSONArray v10Array = msg["v10"].AsArray;
        _v10Dic = new Dictionary<string, float>();

        for (int i = 0; i < v10Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v10Dic.Add(meshName, v10Array[i].AsFloat);
        }

        //Parse v11 of the rotation matrix values with the correponding name as string
        JSONArray v11Array = msg["v11"].AsArray;
        _v11Dic = new Dictionary<string, float>();

        for (int i = 0; i < v11Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v11Dic.Add(meshName, v11Array[i].AsFloat);
        }

        //Parse v12 of the rotation matrix values with the correponding name as string
        JSONArray v12Array = msg["v12"].AsArray;
        _v12Dic = new Dictionary<string, float>();

        for (int i = 0; i < v12Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v12Dic.Add(meshName, v12Array[i].AsFloat);
        }
        //Parse v13 of the rotation matrix values with the correponding name as string
        JSONArray v13Array = msg["v13"].AsArray;
        _v13Dic = new Dictionary<string, float>();

        for (int i = 0; i < v13Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v13Dic.Add(meshName, v13Array[i].AsFloat);
        }

        //Parse v20 of the rotation matrix values with the correponding name as string
        JSONArray v20Array = msg["v20"].AsArray;
        _v20Dic = new Dictionary<string, float>();

        for (int i = 0; i < v20Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v20Dic.Add(meshName, v20Array[i].AsFloat);
        }
        //Parse v21 of the rotation matrix values with the correponding name as string
        JSONArray v21Array = msg["v21"].AsArray;
        _v21Dic = new Dictionary<string, float>();

        for (int i = 0; i < v21Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v21Dic.Add(meshName, v21Array[i].AsFloat);
        }
        //Parse v22 of the rotation matrix values with the correponding name as string
        JSONArray v22Array = msg["v22"].AsArray;
        _v22Dic = new Dictionary<string, float>();

        for (int i = 0; i < v22Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v22Dic.Add(meshName, v22Array[i].AsFloat);
        }
        //Parse v23 of the rotation matrix values with the correponding name as string
        JSONArray v23Array = msg["v23"].AsArray;
        _v23Dic = new Dictionary<string, float>();

        for (int i = 0; i < v23Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v23Dic.Add(meshName, v23Array[i].AsFloat);
        }
        //Parse v30 of the rotation matrix values with the correponding name as string
        JSONArray v30Array = msg["v30"].AsArray;
        _v30Dic = new Dictionary<string, float>();

        for (int i = 0; i < v30Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v30Dic.Add(meshName, v30Array[i].AsFloat);
        }
        //Parse v31 of the rotation matrix values with the correponding name as string
        JSONArray v31Array = msg["v31"].AsArray;
        _v31Dic = new Dictionary<string, float>();

        for (int i = 0; i < v31Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v31Dic.Add(meshName, v31Array[i].AsFloat);
        }
        //Parse v32 of the rotation matrix values with the correponding name as string
        JSONArray v32Array = msg["v32"].AsArray;
        _v32Dic = new Dictionary<string, float>();

        for (int i = 0; i < v32Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v32Dic.Add(meshName, v32Array[i].AsFloat);
        }
        //Parse v33 of the rotation matrix values with the correponding name as string
        JSONArray v33Array = msg["v33"].AsArray;
        _v33Dic = new Dictionary<string, float>();

        for (int i = 0; i < v33Array.Count; i++)
        {
            string meshName = nameArray[i].ToString().Replace("\"", string.Empty);
            _v33Dic.Add(meshName, v33Array[i].AsFloat);
        }
    }

    public RoboyPoseMsg(Dictionary<string, int> nameIndexDic, Dictionary<string, float> xDic, Dictionary<string, float> yDic, Dictionary<string, float> zDic, 
        Dictionary<string, float> rollDic, Dictionary<string, float> pitchDic, Dictionary<string, float> yawDic, 
        Dictionary<string, float> qxDic, Dictionary<string, float> qyDic, Dictionary<string, float> qzDic, Dictionary<string, float> qwDic)
    {
        _nameIndexDic = nameIndexDic;
        _xDic = xDic;
        _yDic = yDic;
        _zDic = zDic;
        _rollDic = rollDic;
        _pitchDic = pitchDic;
        _yawDic = yawDic;
        _qxDic = qxDic;
        _qyDic = qyDic;
        _qzDic = qzDic;
        _qwDic = qwDic;
    }

    public static string GetMessageType()
    {
        return "common_utilities/Pose";
    }

    public override string ToString()
    {
        return "common_utilities/Pose [name =";
    }

    public override string ToYAMLString()
    {
        return base.ToYAMLString();
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion //PRIVATE_METHODS
}
