using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class SelectionWheelManager : MonoBehaviour
{

    private bool visible = false;
    private bool stillVisible = false;
    private double curSpin = 0;
    private float curAngle = 0;
    private int elems;
    private int selected = -1;
    public float speed;
    public float threshold;
    public Canvas canvas;

    // Initialize Wheel: Place options
    void Start()
    {
        Vector2 radius;
        RectTransform t = GetComponent<RectTransform>();
        radius.x = t.rect.x * t.localScale.y; //assume square
        radius.y = 0;
        Debug.Log(radius);
        Component[] children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        elems = children.Length - 1; //ignore parent
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject)
            {
                Debug.Log(children[i].name);
                Vector2 offset = Quaternion.Euler(0, 0, 360 / elems * (i - 1)) * radius; //start from 0
                Debug.Log(offset);
                transform.Translate(offset.x, offset.y, 0);
            }
        }
        canvas.GetComponent<Canvas>().enabled = false;
        visible = false;
        HighlightSelection();
    }

    // Update is called once per frame
    void Update()
    {

        // update the current spin with the mouse wheel
        double friction;
        Component[] children;

        curSpin += Input.GetAxis("Wheel") * speed * Time.deltaTime;
        if (curSpin > -threshold && curSpin < threshold && Input.GetAxis("Wheel") == 0)
        {
            curSpin = 0;
            stillVisible = false;
            StartCoroutine(DisableCanvas());
        }
        if (Input.GetAxis("Wheel") != 0)
        {
            stillVisible = true;
            if (!visible) EnableCanvas();
        }
        curAngle = transform.rotation.eulerAngles.z;
        friction = curSpin * Time.deltaTime;
        curSpin -= (float)friction;
        // finally, do the actual rotation
        transform.Rotate(Vector3.back * (float)curSpin, Space.World);
        //counter rotate children
        children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject)
            {
                transform.Rotate(Vector3.back * (float)(-curSpin), Space.Self);
            }
        }
        HighlightSelection();
    }
    void HighlightSelection()
    {
        int step = (360 / elems);
        int tmp = elems - (int)(curAngle) / step - 1;// map selection from 0-3
        tmp = (tmp + 3) % 4;
        if (tmp == selected)
        {
            return;
        }
        selected = tmp;
        Debug.Log("selection: " + selected);
        Text[] texts = GetComponentsInChildren<Text>();
        Debug.Log(texts.Length);
        for (int i = 0; i < texts.Length; i++)
        {
            if (selected != i)
            {
                texts[i].fontStyle = FontStyle.Normal;
                texts[i].color = Color.gray;

            }
            else
            {
                texts[i].fontStyle = FontStyle.Bold;
                texts[i].color = Color.black;
            }
        }
    }

    void EnableCanvas()
    {
        canvas.GetComponent<Canvas>().enabled = true;
        visible = true;
    }
    IEnumerator DisableCanvas()
    {
        yield return new WaitForSeconds(1);
        if (!stillVisible)
        {
            canvas.GetComponent<Canvas>().enabled = false;
            visible = false;
        }
    }
}

