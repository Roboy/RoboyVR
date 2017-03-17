using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RoboyValueGenerator : Singleton<RoboyValueGenerator>
{
    public float timeStep = 0.2f;

    private List<RoboyPart> m_RoboyParts = new List<RoboyPart>();

    void Start()
    {
        Initialize();
        StartCoroutine(generateValuesCoroutine());
    }

    void Initialize()
    {
        m_RoboyParts = RoboyManager.Instance.RoboyParts.Values.ToList();
        //foreach (var roboyPart in m_RoboyParts)
        //{
        //    foreach (var category in roboyPart.Categories.Values)
        //    {
        //        foreach (var motor in category.Motors)
        //        {
        //            motor.Value.Values = new List<float>(new float[30]);
        //        }
        //    }
        //}
    }

    IEnumerator generateValuesCoroutine()
    {
        while (true)
        {
            yield return new WaitForSeconds(timeStep);
            generateValues();
        }
    }

    void generateValues()
    {
        foreach (var roboyPart in m_RoboyParts)
        {
            foreach (var category in roboyPart.Categories.Values)
            {
                foreach (var motor in category.Motors)
                {
                    float newValue = Random.Range(0, 100f);
                    //Debug.Log("Rand: " + newValue);
                    ExtensionMethods.ShiftRight(motor.Value.Values, 1);
                    motor.Value.Values[0] = newValue;
                }
            }
        }
    }
}
