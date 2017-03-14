using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIPanelPanelMode : MonoBehaviour
{

    public UIPanelPage UIPanelPagePrefab;

	public List<UIPanelPage> PanelPages = new List<UIPanelPage>();

    public ModeManager.Panelmode PanelMode;

    public CanvasGroup CanvasGroup;

    public void CreatePages(int motorCount, int pagesCount, RoboyPart roboyPart)
    {
        if (pagesCount < 1)
        {
            Debug.Log("Pages count cannot be smaller than 1!");
            return;
        }

        int motorCountLeft = motorCount;

        for (int i = 0; i < pagesCount; i++)
        {
            UIPanelPage panelPage = Instantiate(UIPanelPagePrefab, Vector3.zero, Quaternion.identity);
            panelPage.transform.SetParent(transform, false);
            panelPage.CreateGraphRenderers(motorCountLeft, i, PanelMode, roboyPart);
            PanelPages.Add(panelPage);
            panelPage.gameObject.SetActive(false);
            motorCountLeft = motorCountLeft - 4;
        }
    }
}
