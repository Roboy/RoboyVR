using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIPanelPage : MonoBehaviour
{

    public GraphRenderer GraphRendererPrefab;

    [HideInInspector]
	public List<GraphRenderer> GraphRenderers = new List<GraphRenderer>();

    //public void Initialize(List<float> values)
    //{
    //    transform.GetComponentsInChildren()
    //}

    public void CreateGraphRenderers(int motorCountLeft)
    {
        int motorsToCreate = Mathf.Min(4, motorCountLeft);

        for (int i = 0; i < motorsToCreate; i++)
        {
            GraphRenderer graphRenderer = Instantiate(GraphRendererPrefab, Vector3.zero, Quaternion.identity);
            graphRenderer.transform.SetParent(transform, false);
            GraphRenderers.Add(graphRenderer);
        }
    }
}
