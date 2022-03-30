using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class TestSpawner : MonoBehaviour
{
    public GameObject PrefabAgent;
    public int SpawnCount = 1000;
    public float SpawnRadius = 10;
    public float noiseScale = 1.0f;
    public static float NoiseScale;
    
    public List<GameObject> SpawnedItemsList = new List<GameObject>();

    // Start is called before the first frame update
    void OnEnable()
    {
        for (int i = 0; i < SpawnCount; i++)
        {
            Vector3 circlePos = Random.insideUnitSphere * SpawnRadius;
            
            Vector3 pos = new Vector3(transform.position.x + circlePos.x, transform.position.y + circlePos.y, transform.position.z + circlePos.z);

            
            
            GameObject agentInstance = Instantiate(PrefabAgent, transform);
            agentInstance.transform.position = pos;
            
            SpawnedItemsList.Add(agentInstance);
        }
    }

    private void OnDisable()
    {
        foreach (GameObject agentInstance in SpawnedItemsList)
        {
            DestroyImmediate(agentInstance);
        }
        SpawnedItemsList.Clear();
    }

    // Update is called once per frame
    void Update()
    {
        NoiseScale = noiseScale;
    }
}