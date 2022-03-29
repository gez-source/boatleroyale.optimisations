using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

// Cams mostly hack buoyancy
public class Buoyancy : MonoBehaviour
{
	public float splashVelocityThreshold;
	public float forceScalar;
	public float waterLineHack; // HACK

	public int underwaterVerts;
	public float dragScalar;

	public static event Action<GameObject, Vector3, Vector3> OnSplash;
	public static event Action<GameObject> OnDestroyed;

	Vector3 worldVertPos;

	// Gerallt
	public int underwaterVerts2;
	private Rigidbody rb;
	private MeshFilter meshFilter;
	private Transform _transform;
	private Vector3 forceAmount;
	private Vector3 forcePosition;
	private float dt;
	private Matrix4x4 transformMatrix;
	private Quaternion transformRotation;

	private Vector3 netForce; // Force for thread 1
	private Vector3 netTorque;

	private static bool loaded = false;
	private static int vertsLength;
	private static int normalsCount;
	private static Vector3[] verts;
	private static Vector3[] normals;
	private static Mesh mesh;
	private static Vector3 worldPosition;
	private static Vector3 localPosition;
	private static Vector3[] vertexWorldPositionsLUT;
	private static Vector3[] normalWorldPositionLUT;
	private static float velocityMagnitude;
	private static float angularVelocityMagnitude;
	private static Vector3 normalWorldPosition;
	
	/// <summary>
	/// The queue of actions to execute on the main thread.
	/// </summary>
	private static ConcurrentQueue<Action> actionsQueue = new ConcurrentQueue<Action>();

	private void Start()
	{
		rb = GetComponent<Rigidbody>();
		_transform = GetComponent<Transform>();
		
		if (!loaded)
		{
			meshFilter = GetComponent<MeshFilter>();
			mesh = meshFilter.mesh;
			normalsCount = mesh.normals.Length;
			vertsLength = mesh.vertices.Length;
			verts = mesh.vertices;
			normals = mesh.normals;
		}
	}

	void Update()
	{
		//CalculateForces();
		CalculateForcesThreaded();
	}

	private void FixedUpdate()
	{
		//CalculateForces();
	}
	
	private void CalculateForces()
	{
		underwaterVerts = 0;

		dt = Time.deltaTime;
		
		worldPosition = _transform.position;
		localPosition = _transform.localPosition;
		transformRotation = _transform.rotation;
		velocityMagnitude = rb.velocity.magnitude;
		angularVelocityMagnitude = rb.angularVelocity.magnitude;

		for (var index = 0; index < normalsCount; index++)
		{
			worldVertPos = worldPosition + _transform.TransformDirection(verts[index]);

			if (worldVertPos.y < waterLineHack)
			{
				// Splashes only on surface of water plane
				if (worldVertPos.y > waterLineHack - 0.1f)
				{
					if (velocityMagnitude > splashVelocityThreshold || angularVelocityMagnitude > splashVelocityThreshold)
					{
						//print(velocityMagnitude); // Gerallt: Slow to Debug.Log
						if (OnSplash != null)
						{
							OnSplash.Invoke(gameObject, worldVertPos, rb.velocity);
						}
					}
				}
				
				forceAmount = (_transform.TransformDirection(-normals[index]) * forceScalar) * dt;
				forcePosition = worldPosition + _transform.TransformDirection(verts[index]);
				
				rb.AddForceAtPosition(forceAmount, forcePosition, ForceMode.Force);
				
				underwaterVerts++;
			}
			
			// HACK to remove sunken boats
			if (worldVertPos.y < waterLineHack - 10f)
			{
				DestroyParentGO();
				break;
			}
		}
		
		// Drag for percentage underwater
		rb.drag = (underwaterVerts / (float)vertsLength) * dragScalar;
		rb.angularDrag = (underwaterVerts / (float)vertsLength) * dragScalar;
	}
	
	private void CalculateForcesThreaded()
	{
		worldPosition = _transform.position;
		localPosition = _transform.localPosition;
		transformRotation = _transform.rotation;
		velocityMagnitude = rb.velocity.magnitude;
		angularVelocityMagnitude = rb.angularVelocity.magnitude;
		
		dt = Time.deltaTime;
		transformMatrix = _transform.localToWorldMatrix;

		// Divide the work amongst two threads:
		CalculateForcesThreadedAsync();
		
		// Execute any scheduled actions on the main thread.
		if(!actionsQueue.IsEmpty)
		{
			while(actionsQueue.TryDequeue(out var action))
			{
				action?.Invoke();
			}
		}
	}

	private void CalculateForcesThreadedAsync()
	{
		int quarter = normalsCount / 4;
		
		var task1 = Task.Run(() => CalculateForcesThreaded(0, quarter));
		var task2 = Task.Run(() => CalculateForcesThreaded(quarter, 2 * quarter));
		var task3 = Task.Run(() => CalculateForcesThreaded(2 * quarter,  3 * quarter));
		var task4 = Task.Run(() => CalculateForcesThreaded(3 * quarter, normalsCount));
		
		//Task.WaitAll(task1);
		Task.WaitAll(task1, task2, task3, task4);

		PhysicsResult physicsResult = task1.Result;
		PhysicsResult physicsResult2 = task2.Result;
		PhysicsResult physicsResult3 = task3.Result;
		PhysicsResult physicsResult4 = task4.Result;
		
		if (rb != null)
		{
			int underwaterVertsCount = physicsResult.UnderwaterVerts 
			                           + physicsResult2.UnderwaterVerts
			                           + physicsResult3.UnderwaterVerts
			                           + physicsResult4.UnderwaterVerts
			                           ;
			
			// Drag for percentage underwater
			rb.drag = ((underwaterVertsCount) / (float)vertsLength) * dragScalar;
			rb.angularDrag = ((underwaterVertsCount) / (float)vertsLength) * dragScalar;
			//rb.drag = ((physicsResult.UnderwaterVerts) / (float)vertsLength) * dragScalar;
			//rb.angularDrag = ((physicsResult.UnderwaterVerts) / (float)vertsLength) * dragScalar;
			
			// Apply calculated net forces to rigidbody 
			rb.AddForce(physicsResult.NetForce, ForceMode.Force);
			rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
			
			rb.AddForce(physicsResult2.NetForce, ForceMode.Force);
			rb.AddTorque(physicsResult2.NetTorque, ForceMode.Force);
			
			rb.AddForce(physicsResult3.NetForce, ForceMode.Force);
			rb.AddTorque(physicsResult3.NetTorque, ForceMode.Force);
			
			rb.AddForce(physicsResult4.NetForce, ForceMode.Force);
			rb.AddTorque(physicsResult4.NetTorque, ForceMode.Force);
		}

	}

	public class PhysicsResult
	{
		public Vector3 NetForce;
		public Vector3 NetTorque;
		public int UnderwaterVerts;
	}
	
	private PhysicsResult CalculateForcesThreaded(int startIndex, int endIndex)
	{
		PhysicsResult result = new PhysicsResult();

		for (var index = startIndex; index < endIndex; index++)
		{
			worldVertPos = worldPosition + TransformDirection(transformRotation, verts[index]);

			if (worldVertPos.y < waterLineHack)
			{
				// Splashes only on surface of water plane
				if (worldVertPos.y > waterLineHack - 0.1f)
				{
					if (velocityMagnitude > splashVelocityThreshold || angularVelocityMagnitude > splashVelocityThreshold)
					{
						if (OnSplash != null)
						{
							OnSplash.Invoke(gameObject, worldVertPos, rb.velocity);
						}
					}
				}
				
				forceAmount = (TransformDirection(transformRotation,-normals[index]) * forceScalar) * dt;
				forcePosition = worldPosition + TransformDirection(transformRotation, verts[index]);
				
				//rb.AddForceAtPosition(forceAmount, forcePosition, ForceMode.Force);
				netForce += forceAmount;
				
				//Torque is the cross product of the distance to center of mass and the force vector.
				netTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);
				
				underwaterVerts++;
			}
			
			// HACK to remove sunken boats
			if (worldVertPos.y < waterLineHack - 10f)
			{
				actionsQueue.Enqueue(() =>
				{
					// This code will run on the main thread
					
					DestroyParentGO();
				});
				break;
			}
		}

		result.UnderwaterVerts = underwaterVerts;
		result.NetForce = netForce;
		result.NetTorque = netTorque;

		return result;
	}

	private void DestroyParentGO()
	{
		if (OnDestroyed != null)
		{
			OnDestroyed.Invoke(gameObject);
		}
		Destroy(gameObject);
		rb = null;
	}

	static Vector3 TransformDirection(Quaternion rotation, Vector3 direction)
	{
		return rotation * direction;
	}
}
