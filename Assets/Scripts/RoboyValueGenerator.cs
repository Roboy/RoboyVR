using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RoboyValueGenerator : Singleton<RoboyValueGenerator>
{

    private List<RoboyPart> m_RoboyParts = new List<RoboyPart>();

    void Initialize()
    {
        m_RoboyParts = RoboyManager.Instance.RoboyParts.Values.ToList();

        foreach (var roboyPart in m_RoboyParts)
        {
            foreach (var category in roboyPart.Categories.Values)
            {
                foreach (var motor in category.Motors)
                {
                    motor.Value.Values = new List<float>(new float[30]);
                }
            }
        }
    }

    void generateValues()
    {
        
    }
}
