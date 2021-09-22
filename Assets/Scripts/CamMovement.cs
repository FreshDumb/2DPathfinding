using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamMovement : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.A))
        {
            gameObject.transform.Translate(Vector2.left * 1 * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.D))
        {
            gameObject.transform.Translate(Vector2.right * 1 * Time.deltaTime);

        }

        if (Input.GetKey(KeyCode.W))
        {
            gameObject.transform.Translate(Vector2.up * 1 * Time.deltaTime);

        }
        else if (Input.GetKey(KeyCode.S))
        {
            gameObject.transform.Translate(Vector2.down * 1 * Time.deltaTime);

        }
    }
}
