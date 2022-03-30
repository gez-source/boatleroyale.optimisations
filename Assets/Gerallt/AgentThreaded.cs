using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

public class AgentThreaded : MonoBehaviour
{
    public float movementSpeed = 0.1f;

    public Vector3 position = Vector3.zero;
    
    // Start is called before the first frame update
    void Start()
    {
        Thread thread = new Thread(MoveGuyThread);
        thread.Start();
    }

    public void MoveGuyThread()
    {
        while (true)
        {
            Vector3 pos = position;

            float dither = Mathf.Clamp01(Mathf.PerlinNoise((pos.y + Time.time) * TestSpawner.NoiseScale, 0));

            float direction = ((dither) * 2) - 0.5f; // Between -1 and 1
        
            float y =  direction * movementSpeed;
        
            //Debug.Log(y);
        
            pos.y = pos.y + y;
        
            position = pos;
        }
    }
    
    // Update is called once per frame
    void Update()
    {


        transform.position = position;
    }
}
