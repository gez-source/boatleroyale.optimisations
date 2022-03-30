using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Agent : MonoBehaviour
{
    public float movementSpeed = 0.1f;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 pos = transform.position;

        float dither = Mathf.Clamp01(Mathf.PerlinNoise((pos.y + Time.time) * TestSpawner.NoiseScale, 0));

        float direction = ((dither) * 2) - 0.5f; // Between -1 and 1
        
        float y =  direction * movementSpeed;
        
        //Debug.Log(y);
        
        pos.y = pos.y + y;
        
        transform.position = pos;
    }
}
