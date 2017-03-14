using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;

public class RoboyPart : MonoBehaviour
{
    [HideInInspector]
    public Vector3 Position;
    [HideInInspector]
    public Quaternion Rotation;
    [HideInInspector]
    public List<Category> Categories = new List<Category>();

    [HideInInspector] public int MotorCount;

    public void Initialize(int count)
    {
        var enumList = Enum.GetValues(typeof(ModeManager.Panelmode));

        foreach (var enumElem in enumList)
        {
            Categories.Add(new Category((ModeManager.Panelmode)enumElem, count));
        }

        MotorCount = count;
    }
}

public class Category
{
    public ModeManager.Panelmode Mode;

    public List<Motor> Motors = new List<Motor>();

    public Category(ModeManager.Panelmode mode, int count)
    {
        Mode = mode;
        initializeMotors(count);
    }

    private void initializeMotors(int count)
    {
        for (int i = 0; i < count; i++)
        {
            Motor motor = new Motor {Name = "Motor" + i };
            motor.Values = new List<float>(new float[30]);
            Motors.Add(motor);
        }
    }
}

public class Motor
{
    public string Name;
    public List<float> Values = new List<float>();
}
