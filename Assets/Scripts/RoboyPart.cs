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

    [HideInInspector] public Dictionary<ModeManager.Panelmode, Category> Categories =
        new Dictionary<ModeManager.Panelmode, Category>();
    
    /// <summary>
    /// number of motors of that part of roboy
    /// </summary>
    [HideInInspector] public int MotorCount;

    public void Initialize(int count)
    {
        var enumList = Enum.GetValues(typeof(ModeManager.Panelmode));

        foreach (var enumElem in enumList)
        {
            Categories.Add((ModeManager.Panelmode)enumElem, new Category((ModeManager.Panelmode)enumElem, count));
        }

        MotorCount = count;
    }
}

public class Category
{
    public ModeManager.Panelmode Mode;

    public Dictionary<string, Motor> Motors = new Dictionary<string, Motor>();

    public Category(ModeManager.Panelmode mode, int count)
    {
        Mode = mode;
        initializeMotors(count);
    }

    private void initializeMotors(int count)
    {
        for (int i = 0; i < count; i++)
        {
            string motorName = "Motor" + i;
            Motor motor = new Motor {Name =  motorName};

            List<float> randomValues = new List<float>();
            for (int j = 0; j < 30; j++)
            {
                float value = UnityEngine.Random.Range(0, 100);
                randomValues.Add(value);
            }

            motor.Values = randomValues;
            Motors.Add(motorName, motor);
        }
    }
}

public class Motor
{
    public string Name;
    public List<float> Values = new List<float>();
}
