using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Reflection;
using System.Threading;

public static class ExtensionMethods {

    /// <summary>
    /// Extension method to do a right shift of a generic list about the given number.
    /// Be aware that the first "shifts" elements are the default ones of the given type and not the previous last elements.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="lst"></param>
    /// <param name="shifts">The number of shifts you want.</param>
    public static void ShiftRight<T>(this List<T> lst, int shifts)
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

    /// <summary>
    /// Extension method which uses <see cref="GetCopyOf{T}(Component, T)" to add an exact copy of the given component/> 
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="go"></param>
    /// <param name="toAdd"></param>
    /// <returns></returns>
    public static T AddComponent<T>(this GameObject go, T toAdd) where T : Component
    {
        return go.AddComponent<T>().GetCopyOf(toAdd) as T;
    }

    /// <summary>
    /// Returns a copy of the exact component.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="comp"></param>
    /// <param name="other"></param>
    /// <returns></returns>
    public static T GetCopyOf<T>(this Component comp, T other) where T : Component
    {
        System.Type type = comp.GetType();
        if (type != other.GetType()) return null; // type mis-match
        BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Default | BindingFlags.DeclaredOnly;
        PropertyInfo[] pinfos = type.GetProperties(flags);
        foreach (var pinfo in pinfos)
        {
            if (pinfo.CanWrite)
            {
                try
                {
                    pinfo.SetValue(comp, pinfo.GetValue(other, null), null);
                }
                catch { } // In case of NotImplementedException being thrown. For some reason specifying that exception didn't seem to catch it, so I didn't catch anything specific.
            }
        }
        FieldInfo[] finfos = type.GetFields(flags);
        foreach (var finfo in finfos)
        {
            finfo.SetValue(comp, finfo.GetValue(other));
        }
        return comp as T;
    }
}
