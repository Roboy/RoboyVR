using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class ExtensionMethods {

    public static void ShiftRight<T>(List<T> lst, int shifts)
    {
        if (lst == null)
        {
            Debug.Log("List is null");
            return;
        }
        if (lst.Count == 0)
        {
            Debug.Log("List is empty!");
            return;
        }

        for (int i = lst.Count - shifts - 1; i >= 0; i--)
        {
            lst[i + shifts] = lst[i];
        }

        for (int i = 0; i < shifts; i++)
        {
            lst[i] = default(T);
        }
    }

    public static T GetComponentInChildWithTag<T>(this GameObject parent, string tag) where T : Component
    {
        Transform t = parent.transform;
        foreach (Transform tr in t)
        {
            if (tr.tag == tag)
            {
                return tr.GetComponent<T>();
            }
        }

        return null;
    }
}
