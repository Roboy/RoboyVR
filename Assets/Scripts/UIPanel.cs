using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIPanel : MonoBehaviour {

    public int Index = -1;

    public enum Side
    {
        Left,
        Top,
        Right
    }

    public Side Alignment = Side.Left;
}
