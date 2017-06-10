using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CameraPanel : MonoBehaviour {

    /// <summary>
    /// Image which size that is to be modified, wrapped in a RectTransform.
    /// </summary>
    public RectTransform Rt;

    /// <summary>
    /// Slider to change the size of the RectTransform.
    /// </summary>
    public Slider Sl;

    [SerializeField]
    private float m_AspectRatio;
    private float m_Scale = 0.5f;
    private float m_Width_Ref;




    // Use this for initialization
    void Start () {
    m_Width_Ref = Rt.rect.width;
    m_AspectRatio = Rt.rect.width / Rt.rect.height;
}
	
	// Update is called once per frame
	void Update () {

        Sl.onValueChanged.AddListener(delegate { ValueChangeCheck(); });

    }

    public CameraPanel(RectTransform rect, Slider slider)
    {
        Rt = rect;
        Sl = slider;
        m_AspectRatio = Rt.rect.width / Rt.rect.height;
        m_Scale = 0.5f;
        m_Width_Ref = Rt.rect.width;
    }


    private void ValueChangeCheck()
    {
        float widthTmp = Rt.rect.width;
        float heightTmp = Rt.rect.height;
        float scaleDiff = 0.0f;

        Rt.sizeDelta = new Vector2(m_Width_Ref * (Sl.value + 0.5f), m_Width_Ref * (Sl.value + 0.5f) / m_AspectRatio);


        //if (m_Scale < Sl.value)
        //{
        //    scaleDiff = Sl.value - m_Scale;
        //    Rt.sizeDelta = new Vector2(m_Width_Ref + widthTmp*scaleDiff, (m_Width_Ref + widthTmp * scaleDiff) / m_AspectRatio );
        
            
        //}
        //if (m_Scale > Sl.value)
        //{
        //    scaleDiff = m_Scale - Sl.value;
        //    Rt.sizeDelta = new Vector2(m_Width_Ref - widthTmp * scaleDiff, (m_Width_Ref - widthTmp * scaleDiff) / m_AspectRatio);
            
        //}

    }
}
