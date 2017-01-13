using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

public class TestTendons : MonoBehaviour
{

    [SerializeField] private GameObject Arrow;
    private GameObject arrowCopy;

    //TEST
    private float m_Timer = 0.05f;
    private float m_currentTime = 0f;
    private LineRenderer Oscillator;
    //TESTEND
	
        // Use this for initialization
	void Start ()
	{
	    drawTendons();
        GameObject OscillatorGO = new GameObject();
        Oscillator = OscillatorGO.AddComponent<LineRenderer>();
        Oscillator.numPositions = 30;
        Oscillator.startWidth = Oscillator.endWidth = 0.1f;

        List<Vector3> lv = new List<Vector3>();

        for (int i = 0; i < Oscillator.numPositions; i++)
        {
            Vector3 pos = new Vector3(i, Random.Range(1f, 10f), -5);
            lv.Add(pos);
        }

        Oscillator.SetPositions(lv.ToArray());
    }
	
	// Update is called once per frame
	void Update ()
	{
        float scale = Random.Range(1f, 2f);

        arrowCopy.transform.localScale = new Vector3(arrowCopy.transform.localScale.x, scale, arrowCopy.transform.localScale.z);

        Material mat = arrowCopy.GetComponent<Renderer>().material;
        if (scale < 1.5f)
            mat.color = Color.Lerp(Color.green, Color.yellow, scale - 0.5f);
        else if (scale > 1.5)
            mat.color = Color.Lerp(Color.yellow, Color.red, scale - 1.5f);

        Color col = mat.color;

        if (col.r > col.g && col.r > col.b)
            col.r = 1f;
        else if (col.g > col.r && col.g > col.b)
            col.g = 1f;
        else if (col.b > col.g && col.b > col.r)
            col.b = 1f;

        mat.color = col;

	    if (m_currentTime > m_Timer)
	    {
            updateForcePath();
	        m_currentTime = 0f;
	    }
        else
	        m_currentTime += Time.deltaTime;
	}

    void drawTendons()
    {
        Dictionary<int, List<Vector3>> tendonsDictionary = new Dictionary<int, List<Vector3>>();
        int tendonsCount = 1;
        int tendonsLength = 10; //UnityEngine.Random.Range(3, 3);

        for (int i = 0; i < tendonsCount; i++)
        {
            List<Vector3> lv = new List<Vector3>();
            for (int j = 0; j < tendonsLength; j++)
            {
                Vector3 position = new Vector3(UnityEngine.Random.Range(-10.0f, 10.0f), UnityEngine.Random.Range(-10.0f, 10.0f), UnityEngine.Random.Range(-10.0f, 10.0f));
                lv.Add(position);
            }
            tendonsDictionary.Add(i, lv);
            //tendonsLength = UnityEngine.Random.Range(2, 10);
        }

        foreach (KeyValuePair<int, List<Vector3>> t in tendonsDictionary)
        {
            //Get number of points for linerenderer
            int points = t.Value.Count;

            GameObject g = new GameObject();
            g.AddComponent<LineRenderer>();
            LineRenderer lr = g.GetComponent<LineRenderer>();
            lr.numPositions = points;
            lr.SetPositions(t.Value.ToArray());
            lr.startWidth = lr.endWidth = 0.1f;
        }

        foreach (KeyValuePair<int, List<Vector3>> t in tendonsDictionary)
        {
            int points = t.Value.Count;

            
            int index = points/2;

            Debug.Log(points);

            Vector3 arrowPos = t.Value[index];

            arrowCopy = Instantiate(Arrow, arrowPos, Quaternion.identity);

            float scale = Random.Range(1f, 2f);

            arrowCopy.transform.localScale = new Vector3(arrowCopy.transform.localScale.x, scale, arrowCopy.transform.localScale.z);

            Material mat = arrowCopy.GetComponent<Renderer>().material;
            if(scale < 1.5f)
                mat.color = Color.Lerp(Color.green, Color.yellow, scale - 1f);
            else if(scale > 1.5)
                mat.color = Color.Lerp(Color.yellow, Color.red, scale - 1f);

            Color col = mat.color;

            if (col.r > col.g && col.r > col.b)
                col.r = 1f;
            else if (col.g > col.r && col.g > col.b)
                col.g = 1f;
            else if (col.b > col.g && col.b > col.r)
                col.b = 1f;

            mat.color = col;
        }
    }

    void drawForcePath()
    {
        float currentForce = arrowCopy.transform.localScale.y;

        GameObject g = new GameObject();
        LineRenderer lr = g.AddComponent<LineRenderer>();
        lr.numPositions = 30;
        lr.startWidth = lr.endWidth = 0.1f;

        List<Vector3> lv = new List<Vector3>();

        for (int i = 0; i < lr.numPositions; i++)
        {
            Vector3 pos = new Vector3(i, Random.Range(1f,10f), -5);
            lv.Add(pos);
        }

        lr.SetPositions(lv.ToArray());
    }

    void updateForcePath()
    {
        List<Vector3> lv = new List<Vector3>();

        for (int i = 0; i < Oscillator.numPositions; i++)
        {
            Vector3 pos = new Vector3(i, Random.Range(1f, 10f), -5);
            lv.Add(pos);
        }

        Oscillator.SetPositions(lv.ToArray());
    }
}
