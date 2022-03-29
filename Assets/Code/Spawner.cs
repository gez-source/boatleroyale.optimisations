
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour
{
	public int count;
	public GameObject prefab;
	public float randomAmount;
	public float randomThrowAmount;

	public float rotationMin = -180;
	public float rotationMax = 180;

	// Use this for initialization
	void Start()
	{
		for (int i = 0; i < count; i++)
		{
			Spawn();
		}
	}

	private void Spawn()
	{
		GameObject o = Instantiate(prefab, transform.position + new Vector3(Random.Range(-randomAmount, randomAmount), Random.Range(-randomAmount, randomAmount)/2f, Random.Range(-randomAmount, randomAmount)), Quaternion.Euler(new Vector3(Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax))));
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
}
