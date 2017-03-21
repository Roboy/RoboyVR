using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

    public void CreateGraphRenderers(int motorCountLeft, int currentPage, ModeManager.Panelmode panelMode, RoboyPart roboyPart)
    {
        int motorsToCreate = Mathf.Min(4, motorCountLeft);

        for (int i = 0; i < motorsToCreate; i++)
        {
            GraphRenderer graphRenderer = Instantiate(GraphRendererPrefab, Vector3.zero, Quaternion.identity);
            graphRenderer.transform.SetParent(transform, false);

            string motorIndex = "Motor" + (i + currentPage*4);

            graphRenderer.TextForValueName.text = motorIndex;

            //Debug.Log(motorIndex);

            graphRenderer.Initialize(roboyPart.Categories[panelMode].Motors[motorIndex].Values, 30, 0f);
            graphRenderer.Play();

            GraphRenderers.Add(graphRenderer);
        }
    }
}
