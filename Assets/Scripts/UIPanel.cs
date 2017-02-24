using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIPanel : MonoBehaviour {

    public int Index = -1;

    public enum Side
    {
        Left,
        Top,
        Right
    }

    public Side Alignment = Side.Left;

    public RoboyPart AssignedRoboyPart;

    public Text Page;

    public CanvasGroup CanvasGroup;

    public List<UIPanelGraphGroup> PanelGraphGroups = new List<UIPanelGraphGroup>();
}
