using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Light))]
public class ZEDLight : MonoBehaviour {
   
    [HideInInspector]
    public static List<ZEDLight> s_lights = new List<ZEDLight>();
    [HideInInspector]
    public Light cachedLight;

    [HideInInspector]
    public float interiorCone = 0.1f;
    [HideInInspector]
    public bool cancelLight = false;
	// Use this for initialization
	void OnEnable () {
        if (!s_lights.Contains(this))
        {
            s_lights.Add(this);
            cachedLight = GetComponent<Light>();
        }
        if (cachedLight.type != LightType.Directional)
        {
            cancelLight = false;
        }
    }


    void OnDisable()
    {
        if (s_lights != null)
        {
            s_lights.Remove(this);
        }
    }

    private void OnValidate()
    {
        if (cachedLight != null)
        {
            if (cachedLight.type != LightType.Directional)
            {
                cancelLight = false;
            }
        }
    }

    public bool IsEnabled()
    {
        if (!cachedLight.enabled)
        {
            return false;
        }

        if (cachedLight.type == LightType.Point)
        {
            if (cachedLight.range <= 0 || cachedLight.intensity <= 0)
            {
                return false;
            }
        }


        if (cachedLight.type == LightType.Spot)
        {
            if (cachedLight.range <= 0 || cachedLight.intensity <= 0)
            {
                return false;
            }
        }
        return true;
    }
}
