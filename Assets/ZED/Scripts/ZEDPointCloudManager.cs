using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ZEDPointCloudManager : MonoBehaviour
{
    /// <summary>
    /// This camera will not see the point cloud
    /// </summary>
    public Camera hiddenObjectFromCamera;

    private int numberPoints = 0;
    private sl.ZEDCamera zed;
    private Texture2D XYZTexture;
    private Texture2D colorTexture;


    private RenderTexture XYZTextureCopy = null;
    private RenderTexture ColorTextureCopy = null;


    private Material mat;
    public bool display = true;
    public bool update = true;
    private bool previousUpdate = true;


    void Start()
    {
        zed = sl.ZEDCamera.GetInstance();
    }

    // Update is called once per frame
    void Update()
    {
        if (zed.CameraIsReady)
        {
            if (numberPoints == 0)
            {
                XYZTexture = zed.CreateTextureMeasureType(sl.MEASURE.XYZ);
                colorTexture = zed.CreateTextureImageType(sl.VIEW.LEFT);
                numberPoints = zed.ImageWidth * zed.ImageHeight;


                mat = Resources.Load("Materials/Mat_ZED_PCL") as Material;
                if (mat != null)
                {
                    mat.SetTexture("_XYZTex", XYZTexture);
                    mat.SetTexture("_ColorTex", colorTexture);

                }
            }


        if (!update && previousUpdate != update)
        {
            if (XYZTextureCopy == null)
            {
                XYZTextureCopy = new RenderTexture(XYZTexture.width, XYZTexture.height, 0, RenderTextureFormat.ARGBFloat);
            }

            if (ColorTextureCopy == null)
            {
                ColorTextureCopy = new RenderTexture(colorTexture.width, colorTexture.height, 0, RenderTextureFormat.ARGB32);
            }


          
            Graphics.Blit(XYZTexture, XYZTextureCopy);
            Graphics.Blit(colorTexture, ColorTextureCopy);

            if (mat != null)
            {
                mat.SetTexture("_XYZTex", XYZTextureCopy);
                mat.SetTexture("_ColorTex", ColorTextureCopy);
            }
        }

        if (update && previousUpdate != update && mat != null)
        {
            mat.SetTexture("_XYZTex", XYZTexture);
            mat.SetTexture("_ColorTex", colorTexture);


        }
        previousUpdate = update;
        }
    }


    void OnApplicationQuit()
    {
        mat = null;
        if (XYZTextureCopy != null)
        {
            XYZTextureCopy.Release();
           
        }

        if (ColorTextureCopy != null)
        {
            ColorTextureCopy.Release();
        }
    }

    void OnRenderObject()
    {
        if (mat != null)
        {
            if (hiddenObjectFromCamera == Camera.current) return;


            if (!display) return;
            mat.SetMatrix("_Position", transform.localToWorldMatrix);
            mat.SetPass(0);
            Graphics.DrawProcedural(MeshTopology.Points, 1, numberPoints);
        }
    }

}
