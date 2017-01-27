using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using Valve.VR.InteractionSystem;
using Random = UnityEngine.Random;

public class TestTendons : MonoBehaviour
{
    public int ValuesSize = 30;
    public float MaximumX = 10f;
    public float MaximumY = 2f;
    [SerializeField] private GameObject Arrow;
    private GameObject arrowCopy;

    //TEST
    private float m_Timer = 0.05f;
    private float m_currentTime = 0f;
    private LineRenderer Oscillator;
    private Vector3 m_LastPosition = Vector3.zero;
    private List<Vector3> lr_List = new List<Vector3>();
    private List<float> volt_values = new List<float>();
    private float minValue;
    private float maxValue;
    private float stepSize = 1;

    //TESTEND
	
        // Use this for initialization
	void Start ()
	{
	    //drawTendons();

	    
	    MaximumX = GetComponent<RectTransform>().rect.width * transform.parent.localScale.x;
	    MaximumY = GetComponent<RectTransform>().rect.height * transform.parent.localScale.y;
        initializeVoltPath();
        Debug.Log(MaximumX + "---" + MaximumY);
	    // Oscillator.SetPositions(lv.ToArray());
	}
	
	// Update is called once per frame
	void Update ()
	{
        //float scale = Random.Range(1f, 2f);

        //arrowCopy.transform.localScale = new Vector3(arrowCopy.transform.localScale.x, scale, arrowCopy.transform.localScale.z);

        //Material mat = arrowCopy.GetComponent<Renderer>().material;
        //if (scale < 1.5f)
        //    mat.color = Color.Lerp(Color.green, Color.yellow, scale - 0.5f);
        //else if (scale > 1.5)
        //    mat.color = Color.Lerp(Color.yellow, Color.red, scale - 1.5f);

        //Color col = mat.color;

        //if (col.r > col.g && col.r > col.b)
        //    col.r = 1f;
        //else if (col.g > col.r && col.g > col.b)
        //    col.g = 1f;
        //else if (col.b > col.g && col.b > col.r)
        //    col.b = 1f;

        //mat.color = col;


            updateVoltPath();


	    m_LastPosition = transform.position;
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

            //Debug.Log(points);

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
        GameObject OscillatorGO = new GameObject();
        Oscillator = OscillatorGO.AddComponent<LineRenderer>();
        Oscillator.numPositions = 30;
        Oscillator.startWidth = Oscillator.endWidth = 0.1f;

        GameObject g = new GameObject();
        LineRenderer lr = g.AddComponent<LineRenderer>();
        lr.numPositions = 30;
        lr.startWidth = lr.endWidth = 0.1f;

        for (int i = 0; i < lr.numPositions; i++)
        {
            Vector3 pos = new Vector3(i, Mathf.Sin(i), 0f);
            pos += transform.position - (transform.position - m_LastPosition);
            pos = transform.rotation * pos;
            lr_List.Add(pos);
        }

        lr.SetPositions(lr_List.ToArray());
    }

    void updateForcePath()
    {
        for (int i = 0; i < lr_List.Count; i++)
        {
            if (i == lr_List.Count - 1)
            {
                lr_List.Insert(0, lr_List[lr_List.Count-1]);
                lr_List.RemoveAt(lr_List.Count - 1);
            }
            else
            {
                Vector3 newPos = lr_List[i];
                Vector3 nextPos = lr_List[i + 1];
                newPos += transform.position - (transform.position - m_LastPosition);
                newPos = transform.rotation * newPos;
                nextPos += transform.position - (transform.position - m_LastPosition);
                nextPos = transform.rotation * nextPos;
                lr_List[i] = new Vector3(nextPos.x, newPos.y, newPos.z);
            }   
        }
        Oscillator.SetPositions(lr_List.ToArray());
    }

    void initializeValues()
    {
        for (int i = 0; i < ValuesSize; i++)
        {
            volt_values.Add(Random.Range(0f,10f));
        }
    }

    void initializeVoltPath()
    {
        GameObject OscillatorGO = new GameObject();
        Oscillator = OscillatorGO.AddComponent<LineRenderer>();
        Oscillator.numPositions = ValuesSize;
        Oscillator.startWidth = Oscillator.endWidth = 0.01f;

        initializeValues();

        stepSize = MaximumX/(float) ValuesSize;

        minValue = findSmallestElement(volt_values);
        maxValue = findBiggestElement(volt_values);

        for (int i = 0; i < ValuesSize; i++)
        {
            Vector3 pos = new Vector3(i * stepSize, scaleValues(minValue, maxValue, volt_values[i]), 0);
            pos = transform.rotation*pos;
            pos += transform.position;
            lr_List.Add(pos);
        }

        Oscillator.SetPositions(lr_List.ToArray());

    }

    void updateValues()
    {
        ShiftRight(volt_values, 1);
        float newValue = Random.Range(0, 10f);

        if (newValue > maxValue)
            maxValue = newValue;
        else if (newValue < minValue)
            minValue = newValue;

        volt_values[0] = newValue;
    }

    void updateVoltPath()
    {
        lr_List.Clear();

        updateValues();
        for (int i = 0; i < ValuesSize; i++)
        {
            Vector3 pos = new Vector3(i * stepSize, scaleValues(minValue, maxValue, volt_values[i]), 0);
            pos = transform.rotation * pos;
            pos += transform.position;
            lr_List.Add(pos);
        }

        Oscillator.SetPositions(lr_List.ToArray());
    }

    float findSmallestElement(List<float> list )
    {
        if (list == null)
        {
            throw new ArgumentNullException("self");
        }

        if (list.Count == 0)
        {
            throw new ArgumentException("List is empty.", "self");
        }

        float minima = list[0];
        foreach (float f in list)
        {
            if (f < minima)
            {
                minima = f;
            }
        }

        return minima;
    }

    float findBiggestElement(List<float> list)
    {
        if (list == null)
        {
            throw new ArgumentNullException("self");
        }

        if (list.Count == 0)
        {
            throw new ArgumentException("List is empty.", "self");
        }

        float maxima = list[0];
        foreach (float f in list)
        {
            if (f > maxima)
            {
                maxima = f;
            }
        }

        return maxima;
    }

    float scaleValues(float min, float max, float value)
    {
        return (value - min)*MaximumY/(max - min);
    }

    public static void ShiftRight<T>(List<T> lst, int shifts)
    {
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
