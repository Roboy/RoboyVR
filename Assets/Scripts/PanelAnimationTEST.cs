using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PanelAnimationTEST : MonoBehaviour
{

    private RectTransform m_RectTransform;

    private List<CanvasGroup> m_ChildCanvasGroups = new List<CanvasGroup>();
    private List<BoxCollider> m_ChildBoxColliders = new List<BoxCollider>();

    void Awake()
    {
        m_RectTransform = GetComponent<RectTransform>();

        foreach (Transform t in transform)
        {
            CanvasGroup cG;

            if ((cG = t.GetComponent<CanvasGroup>()) != null)
            {
                m_ChildCanvasGroups.Add(cG);
            }

            BoxCollider bC;

            if ((bC = t.GetComponent<BoxCollider>()) != null)
            {
                m_ChildBoxColliders.Add(bC);
            }
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.A))
        {
            StartCoroutine(shrinkCoroutine());
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            StartCoroutine(enlargeCoroutine());
        }
    }



    IEnumerator shrinkCoroutine()
    {
        float startWidth = m_RectTransform.sizeDelta.x;
        float endWidth = 35f;

        float duration = 0.25f;
        float currentDuration = 0f;

        foreach (BoxCollider bC in m_ChildBoxColliders)
        {
            bC.enabled = false;
        }

        while (currentDuration < duration)
        {

            foreach (var c in m_ChildCanvasGroups)
            {
                c.alpha = Mathf.Lerp(1f, 0f, currentDuration/duration);
            }
            currentDuration += Time.deltaTime;
            yield return null;
        }

        foreach (var c in m_ChildCanvasGroups)
        {
            c.alpha = 0f;
        }

        currentDuration = 0f;

        while (currentDuration < duration)
        {
            m_RectTransform.sizeDelta = new Vector2(Mathf.Lerp(startWidth, endWidth, currentDuration/duration), m_RectTransform.sizeDelta.y);
            currentDuration += Time.deltaTime;
            yield return null;
        }
        m_RectTransform.sizeDelta = new Vector2(endWidth, m_RectTransform.sizeDelta.y);

        StartCoroutine(InputManager.Instance.GUI_Controller.ExpandPanel());



    }

    IEnumerator enlargeCoroutine()
    {
        float startWidth = m_RectTransform.sizeDelta.x;
        float endWidth = 170f;

        float duration = 0.25f;
        float currentDuration = 0f;

        while (currentDuration < duration)
        {
            Debug.Log("asds");

            m_RectTransform.sizeDelta = new Vector2(Mathf.Lerp(startWidth, endWidth, currentDuration / duration), m_RectTransform.sizeDelta.y);
            currentDuration += Time.deltaTime;
            yield return null;
        }
        m_RectTransform.sizeDelta = new Vector2(endWidth, m_RectTransform.sizeDelta.y);
        currentDuration = 0f;

        while (currentDuration < duration)
        {

            foreach (var c in m_ChildCanvasGroups)
            {
                c.alpha = Mathf.Lerp(0f, 1f, currentDuration / duration);
            }
            currentDuration += Time.deltaTime;
            yield return null;
        }

        foreach (var c in m_ChildCanvasGroups)
        {
            c.alpha = 1f;
        }
        
        foreach (BoxCollider bC in m_ChildBoxColliders)
        {
            bC.enabled = true;
        }

        
    }
}
