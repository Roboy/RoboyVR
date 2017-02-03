using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExtensionMethods : MonoBehaviour {

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
}
