using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIPanelPanelMode : MonoBehaviour
{

    public UIPanelPage UIPanelPagePrefab;

	public List<UIPanelPage> PanelPages = new List<UIPanelPage>();

    public ModeManager.Panelmode PanelMode;

    //public Text PageText;

    //public int CurrentPage;

    public CanvasGroup CanvasGroup;

    public void CreatePages(int count)
    {
        if (count < 1)
        {
            Debug.Log("Pages count cannot be smaller than 1!");
            return;
        } 

        for (int i = 0; i < count; i++)
        {
            UIPanelPage panelPage = Instantiate(UIPanelPagePrefab, Vector3.zero, Quaternion.identity);
            panelPage.transform.SetParent(transform, false);
            PanelPages.Add(panelPage);
            panelPage.gameObject.SetActive(false);
        }

        //PanelPages[0].gameObject.SetActive(true);
        //CurrentPage = 1;
        //PageText.text = CurrentPage + "/" + PanelPages.Count;
    }

    //public void ChangePage()
    //{
    //    if (PanelPages.Count > 1)
    //    {
    //        int nextPage = (CurrentPage + 1) % PanelPages.Count;
    //        PanelPages[CurrentPage].gameObject.SetActive(false);
    //        PanelPages[nextPage].gameObject.SetActive(true);
    //        CurrentPage = nextPage;

    //        PageText.text = CurrentPage + "/" + PanelPages.Count;
    //    }
    //}
}
