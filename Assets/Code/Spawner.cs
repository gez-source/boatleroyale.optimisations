
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class Spawner : MonoBehaviour
{
	public int count;
	public GameObject prefab;
	public float randomAmount;
	public float randomThrowAmount;

	public float rotationMin = -180;
	public float rotationMax = 180;

	public bool deterministic = false;
	public int seed = 10000;
	public bool DestroyFallen = true;
	
	public List<GameObject> SpawnedObjects = new List<GameObject>();

	public void Cleanup()
	{
		for (int i = 0; i < SpawnedObjects.Count; i++)
		{
			GameObject go = SpawnedObjects[i];
			DestroyImmediate(go);
			//SpawnedObjects.Remove(go);
		}
		SpawnedObjects.Clear();
	}

	public void SpawnAll()
	{
		if (deterministic)
		{
			Random.InitState(seed);
		}

		//Buoyancy.OnDestroyed += Buoyancy_OnDestroyed;
		Buoyancy.DestroyFallenBoats = DestroyFallen;
		
		Cleanup();
		
		for (int i = 0; i < count; i++)
		{
			Spawn();
		}
	}
	
	// Use this for initialization
	void OnEnable()
	{
		//SpawnAll();
	}
	
	void OnDisable()
	{
		//Buoyancy.OnDestroyed -= Buoyancy_OnDestroyed;

		//Cleanup();
	}
	// private void Buoyancy_OnDestroyed(GameObject obj)
	// {
	// 	SpawnedObjects.Remove(obj);
	// }

	private void Spawn()
	{
		GameObject o = Instantiate(prefab, transform.position + new Vector3(Random.Range(-randomAmount, randomAmount), Random.Range(-randomAmount, randomAmount)/2f, Random.Range(-randomAmount, randomAmount)), Quaternion.Euler(new Vector3(Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax))));
		
		SpawnedObjects.Add(o);

		if (Random.value > 0.3f)
		{
			o.GetComponent<Rigidbody>().velocity = new Vector3(Random.Range(-randomThrowAmount, randomThrowAmount), 0,
				Random.Range(-randomThrowAmount, randomThrowAmount));
		}

		if (Random.value > 0.5f)
		{
			o.GetComponent<Rigidbody>().angularVelocity = new Vector3(Random.Range(-randomThrowAmount, randomThrowAmount), Random.Range(-randomThrowAmount, randomThrowAmount), Random.Range(-randomThrowAmount, randomThrowAmount));
		}
	}

	// Update is called once per frame
	void Update()
	{
		if (Input.GetKey(KeyCode.Space))
		{
			Spawn();
		}
	}
	
	void OnGUI()
	{
		GUI.Label(new Rect(100, 10, 300, 20),  SpawnedObjects.Count.ToString() + " boats");
	}
}
