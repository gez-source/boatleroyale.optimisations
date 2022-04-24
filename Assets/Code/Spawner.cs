
using System;
using System.Collections;
using System.Collections.Generic;
using Gerallt;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

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

	private Buoyancy boatFirst;
	private Rigidbody boatFirstRB;
	
	private bool isRunningParallelJob = false;
	private BoatPhysicsJobSurfacesParallel physicsJobSurfacesParallel;
	private JobHandle physicsJobHandleSurfacesParallel;
	private bool bCalculateForcesSimdParallel2LateUpdate = false;
	private TransformAccessArray? transformAccessArray;
	private NativeArray<Vector3> velocities;
	private NativeArray<Vector3> angularVelocities;
	
	public void Cleanup()
	{
		for (int i = 0; i < SpawnedObjects.Count; i++)
		{
			GameObject go = SpawnedObjects[i];
			DestroyImmediate(go);
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

		List<Transform> transformsList = new List<Transform>();
		velocities = new NativeArray<Vector3>(count, Allocator.Persistent);
		angularVelocities = new NativeArray<Vector3>(count, Allocator.Persistent);
		
		for (int i = 0; i < count; i++)
		{
			GameObject go = Spawn(i);

			transformsList.Add(go.transform);
			SpawnedObjects.Add(go);
		}

		boatFirst = SpawnedObjects[0].GetComponent<Buoyancy>();
		boatFirstRB = boatFirst.GetComponent<Rigidbody>();
		
		if (transformAccessArray.HasValue)
		{
			transformAccessArray.Value.Dispose();
		}
		transformAccessArray = new TransformAccessArray(transformsList.ToArray());
	}

	private void OnDestroy()
	{
		velocities.Dispose();
		angularVelocities.Dispose();
		
		if (transformAccessArray.HasValue)
		{
			transformAccessArray.Value.Dispose();
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

	private GameObject Spawn(int boatIndex)
	{
		GameObject o = Instantiate(prefab, transform.position + new Vector3(Random.Range(-randomAmount, randomAmount), Random.Range(-randomAmount, randomAmount)/2f, Random.Range(-randomAmount, randomAmount)), Quaternion.Euler(new Vector3(Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax), Random.Range(rotationMin, rotationMax))));
		Rigidbody rb = o.GetComponent<Rigidbody>();
		
		//rb.isKinematic = true; // Disable Rigidbody because we are doing our own physics
		rb.useGravity = false;
		
		Vector3 linearVelocity = Vector3.zero;
		Vector3 angularVelocity = Vector3.zero;
		
		if (Random.value > 0.3f)
		{
			linearVelocity = new Vector3(Random.Range(-randomThrowAmount, randomThrowAmount), 0, Random.Range(-randomThrowAmount, randomThrowAmount));
		}

		if (Random.value > 0.5f)
		{ 
			angularVelocity = new Vector3(Random.Range(-randomThrowAmount, randomThrowAmount), Random.Range(-randomThrowAmount, randomThrowAmount), Random.Range(-randomThrowAmount, randomThrowAmount));
		}
		
		velocities[boatIndex] = linearVelocity;
		angularVelocities[boatIndex] = angularVelocity;
		
		// rb.velocity = linearVelocity;
		// rb.angularVelocity = angularVelocity;

		return o;
	}

	// Update is called once per frame
	void Update()
	{
		// if (Input.GetKey(KeyCode.Space))
		// {
		// 	GameObject go = Spawn(SpawnedObjects.Count);
		// 	
		// 	SpawnedObjects.Add(go);
		// }
		
		CalculateForcesOnSurfaceSIMDParallel();
	}

	private void CalculateForcesOnSurfaceSIMDParallel()
	{
		Buoyancy boat = boatFirst;
		Rigidbody rb = boatFirstRB;
		
		bCalculateForcesSimdParallel2LateUpdate = true;
		
		#if UNITY_EDITOR
			physicsJobSurfacesParallel.isRunning = EditorApplication.isPlayingOrWillChangePlaymode;
		#else
			physicsJobSurfacesParallel.isRunning = true;
		#endif

		//physicsJobSurfacesParallel.NumBoats = count;
		//physicsJobSurfacesParallel.transformAccessArray = transformAccessArray.Value;
		
		physicsJobSurfacesParallel.mass = rb.mass;
		physicsJobSurfacesParallel.inertiaTensor = rb.inertiaTensor;
		physicsJobSurfacesParallel.aerodynamicDrag = boat.aerodynamicDrag;
		physicsJobSurfacesParallel.dragScalar = boat.dragScalar;
		physicsJobSurfacesParallel.velocities = velocities;
		physicsJobSurfacesParallel.angularVelocities = angularVelocities;
		physicsJobSurfacesParallel.vertsSimd = boat.vertsSimd;
		physicsJobSurfacesParallel.normalsSimd = boat.normalsSimd;
		physicsJobSurfacesParallel.normalsLength = boat.normalsCount;
		physicsJobSurfacesParallel.vertsLength = boat.vertsLength;
		physicsJobSurfacesParallel.polygonAreasSimd = boat.polygonAreasSimd;
		physicsJobSurfacesParallel.facesSimd = boat.facesSimd;
		physicsJobSurfacesParallel.maxSurfaceArea = boat.maxSurfaceArea;
		physicsJobSurfacesParallel.waterLineHack = boat.waterLineHack;
		physicsJobSurfacesParallel.forceScalar = boat.forceScalar;
		physicsJobSurfacesParallel.areaScalar = boat.areaScalar;
		physicsJobSurfacesParallel.waterDepthScalar = boat.waterDepthScalar;
		physicsJobSurfacesParallel.cullAngle = boat.cullAngle;
		physicsJobSurfacesParallel.dontCullTopNormals = boat.dontCullTopNormals;
		physicsJobSurfacesParallel.dt = Time.deltaTime;

		//if (isRunningParallelJob) return;

		//physicsJobHandleSurfacesParallel = physicsJobSurfacesParallel.Schedule();
		physicsJobHandleSurfacesParallel = physicsJobSurfacesParallel.Schedule(transformAccessArray.Value); // SIMD Calculate Forces on Surfaces parallelised   

		isRunningParallelJob = true;
	}
	
	private void LateUpdate()
	{
		if (bCalculateForcesSimdParallel2LateUpdate)
		{
			physicsJobHandleSurfacesParallel.Complete(); // SIMD Calculate Forces SIMD parallelised
		}
	}
	
	void OnGUI()
	{
		GUI.Label(new Rect(400, 10, 300, 20),  SpawnedObjects.Count.ToString() + " boats");
	}
}
