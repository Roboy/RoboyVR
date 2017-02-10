using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;

public class RoboyPart : MonoBehaviour
{
    public Category c;
    private Dictionary<string, Dictionary<string, List<float>>> m_CategoryValuesForEachMotor = new Dictionary<string, Dictionary<string, List<float>>>();

    void Save(string category, Dictionary<string, List<float>> valuesForEachMotor)
    {
        if (m_CategoryValuesForEachMotor.ContainsKey(category))
        {
            m_CategoryValuesForEachMotor[category] = valuesForEachMotor;
        }
    }

    void AddForCategory(string category, string motor, float value)
    {
        if (m_CategoryValuesForEachMotor.ContainsKey(category))
        {
            if (m_CategoryValuesForEachMotor[category].ContainsKey(motor))
            {
                m_CategoryValuesForEachMotor[category][motor].Add(value);
            }
        }
    }

    void Initialize(string name, int maxValues)
    {
        if (name.IsNullOrEmpty())
            name = gameObject.name;

        //Category c = new Category(name, maxValues,);
       // c.MotorValues = motorValues;
    }
}

public class Category
{
    public Category(string name, int maxValues, List<string> motors)
    {
        m_Name = name;
        m_MaxValues = maxValues;

        foreach (string motor in motors)
        {
            List<float> motorValues = new List<float>(new float[maxValues]);
            m_MotorValues.Add(motor, motorValues);
        }
        
    }

    public Dictionary<string, List<float>> MotorValues {
        get { return m_MotorValues; }
    }

    private Dictionary<string, List<float>> m_MotorValues = new Dictionary<string, List<float>>();

    private string m_Name = "";

    private int m_MaxValues = 0;

    public void UpdateValues(string motor, float value)
    {
        if (m_MotorValues.ContainsKey(motor))
        {
            ExtensionMethods.ShiftRight(m_MotorValues[motor], 1);
            m_MotorValues[motor][0] = value;
        }
    }

    public void UpdateValues(string motor, List<float> values)
    {
        if (m_MotorValues.ContainsKey(motor))
        {
            ExtensionMethods.ShiftRight(m_MotorValues[motor], values.Count);
            for (int i = 0; i < values.Count; i++)
            {
                m_MotorValues[motor][i] = values[i];
            }
        }
    }

    public void UpdateValues(Dictionary<string, float> newValueForAllMotors)
    {
        foreach (string key in newValueForAllMotors.Keys)
        {
            if (m_MotorValues.ContainsKey(key))
            {
                ExtensionMethods.ShiftRight(m_MotorValues[key], 1);
                m_MotorValues[key][0] = newValueForAllMotors[key];
            }
        }
    }

    public void UpdateValues(Dictionary<string, List<float>> newValuesForAllMotors)
    {
        foreach (string key in newValuesForAllMotors.Keys)
        {
            if (m_MotorValues.ContainsKey(key))
            {
                ExtensionMethods.ShiftRight(m_MotorValues[key], newValuesForAllMotors[key].Count);
                for (int i = 0; i < newValuesForAllMotors[key].Count; i++)
                {
                    m_MotorValues[key][i] = newValuesForAllMotors[key][i];
                }
            }
        }
    }
}
