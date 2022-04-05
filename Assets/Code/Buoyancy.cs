﻿using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Gerallt;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityStandardAssets.Water;
using static Unity.Mathematics.math;
using Unity.Mathematics;
using float3 = Unity.Mathematics.float3;
using float4 = Unity.Mathematics.float4;

// Cams mostly hack buoyancy
public class Buoyancy : MonoBehaviour
{
    public float splashVelocityThreshold;
    public float forceScalar;
    public float waterLineHack; // HACK

    //public int underwaterVerts;
    public float dragScalar;

    public static event Action<GameObject, Vector3, Vector3> OnSplash;
    public static event Action<GameObject> OnDestroyed;

    Vector3 worldVertPos;

    // Gerallt
    public static bool DestroyFallenBoats;

    private Rigidbody rb;
    private MeshFilter meshFilter;
    private Transform _transform;
    private Vector3 forceAmount;
    private Vector3 forcePosition;
    private float dt;
    private Quaternion transformRotation;
    private int underwaterVertsCount;

    private Task<PhysicsResult> singleTask;
    private List<PhysicsTask> physicsTaskList = new List<PhysicsTask>();
    private Task task1;
    private Task task2;
    private Task task3;
    private Task task4;
    
    private int vertsLength;
    private int normalsCount;
    private int QuarterSize => normalsCount / 4;
    private  Vector3[] verts;
    private Vector3[] normals;
    private Vector3 worldPosition;
    private float velocityMagnitude;
    private float angularVelocityMagnitude;
    private Vector3 normalWorldPosition;

    private Vector3[] forces; // Forces per normal to apply in future
    private Vector3[] torques; // Torques per normal to apply in future

    private float3[] vertsSIMD;
    private float3[] normalsSIMD;
    
    /// <summary>
    /// The queue of actions to execute on the main thread.
    /// </summary>
    private ConcurrentQueue<Action> actionsQueue = new ConcurrentQueue<Action>();

    /// <summary>
    /// Queue updates from other threads to the rigidbody.
    /// </summary>
    private ConcurrentQueue<PhysicsResult> bodyUpdatesQueue = new ConcurrentQueue<PhysicsResult>();
    //private BlockingCollection<PhysicsResult> bodyUpdatesQueue = new BlockingCollection<PhysicsResult>(10);



    private NativeArray<float3> vertsSimd;
    private NativeArray<float3> normalsSimd;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        _transform = GetComponent<Transform>();
        meshFilter = GetComponent<MeshFilter>();
        Mesh mesh = meshFilter.mesh;
        mesh = meshFilter.mesh;
        normalsCount = mesh.normals.Length;
        vertsLength = mesh.vertices.Length;
        verts = mesh.vertices;
        normals = mesh.normals;
        forces = new Vector3[normalsCount];
        torques = new Vector3[normalsCount];
        
        //SetupThreads();
        //CalculateForcesThreadedAsync();
        //PreCalculateForces();
        
        InitSIMDTypes(); // Convert Unity Vectors to Unity.Mathematics SIMD types:
    }

    void Update()
    {
        //CalculateForces();
        CalculateForcesSIMD();
        //CalculateForcesBetter();
        //CalculateForcesSync();
        //CalculateForcesThreaded();
    }

    private void OnDestroy()
    {
        // Dispose of NativeArrays
        vertsSimd.Dispose();
        normalsSimd.Dispose();
    }

    private void FixedUpdate()
    {
        //CalculateForces();
    }

    private void InitSIMDTypes()
    {
        // Convert Unity Vectors to Unity.Mathematics SIMD types:
        int normIdx;
        int vert;

        vertsSimd = new NativeArray<float3>(vertsLength, Allocator.Persistent);
        normalsSimd = new NativeArray<float3>(normalsCount, Allocator.Persistent);
            
        for (vert = 0; vert < vertsLength; vert++)
        {
            Vector3 v = verts[vert];
            
            vertsSimd[vert] = float3(v.x, v.y, v.z);
        }

        for (normIdx = 0; normIdx < normalsCount; normIdx++)
        {
            Vector3 n = normals[normIdx];
            
            normalsSimd[normIdx] = float3(n.x, n.y, n.z);
        }
    }
    
    // private void PreCalculateForces()
    // {
    //     Vector3 centerOfMass = Vector3.zero;
    //     
    //     for (int index = 0; index < normalsCount; index++)
    //     {
    //         forceAmount = -normals[index] * forceScalar;
    //         forcePosition = verts[index];
    //         
    //         forces[index] = forceAmount;
    //         
    //         //Torque is the cross product of the distance to center of mass and the force vector.
    //         torques[index] = Vector3.Cross(forcePosition - centerOfMass, forceAmount);
    //     }
    // }

    // private void OnCollisionStay(Collision collision)
    // {
    //     GameObject go = collision.gameObject;
    //     WaterTile water = go.GetComponent<WaterTile>();
    //
    //     if (water != null)
    //     {
    //         Vector3 NetForce = Vector3.zero;
    //         Vector3 NetTorque = Vector3.zero;
    //         
    //         // Collided with water so need to lookup 
    //         for (int index = 0; index < collision.contactCount; index++)
    //         {
    //             ContactPoint hitPoint = collision.contacts[index];
    //
    //             //Vector3 colliderPosition = hitPoint.thisCollider.ClosestPoint(hitPoint.point);
    //             //Debug.DrawLine(hitPoint.point, hitPoint.normal, Color.red);
    //
    //             forceAmount = -hitPoint.normal * forceScalar;
    //             forcePosition = hitPoint.point;
    //                 
    //             NetForce += forceAmount;
    //
    //             //Torque is the cross product of the distance to center of mass and the force vector.
    //             NetTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);
    //         }
    //
    //         if (rb != null)
    //         {
    //             rb.AddForce(NetForce, ForceMode.Force);
    //             rb.AddTorque(NetTorque, ForceMode.Force);
    //         }
    //     }
    // }

    private void CalculateForcesBetter()
    {
        Vector3 centerOfMass = Vector3.zero;
        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;
        int underwaterVerts = 0;
        Vector3 vertexPosition;
        Vector3 worldVertexPos;
        Vector3 worldPositiion = _transform.position;
        Quaternion rotation = _transform.rotation;

        float dt = Time.deltaTime;
        
        for (int index = 0; index < normalsCount; index++)
        {
            vertexPosition = verts[index];
            worldVertexPos = worldPositiion + (rotation * vertexPosition);

            if (worldVertexPos.y < waterLineHack)
            {
                //forceAmount = ((rotation * -normals[index]) * forceScalar) * dt;
                //forcePosition = worldVertexPos;
                
                forceAmount = ((-normals[index]) * forceScalar) * dt;
                forcePosition = vertexPosition;
                
                netForce += forceAmount;
                //netTorque += Vector3.Cross(forcePosition - centerOfMass, forceAmount);
                netTorque += Vector3.Cross(forcePosition, forceAmount);
                
                underwaterVerts++;
            }
            
            // HACK to remove sunken boats
            if (worldVertexPos.y < waterLineHack - 10f && DestroyFallenBoats)
            {
                DestroyParentGO();
                break;
            }
        }

        if (underwaterVerts > 0 && rb != null)
        {
            // Drag for percentage underwater
            rb.drag = (underwaterVerts / (float) vertsLength) * dragScalar;
            rb.angularDrag = (underwaterVerts / (float) vertsLength) * dragScalar;
        
            // rb.AddForce(netForce, ForceMode.Force);
            // rb.AddTorque(netTorque, ForceMode.Force);
            
            rb.AddRelativeForce(netForce, ForceMode.Force);
            rb.AddRelativeTorque(netTorque, ForceMode.Force);
        }
    }
    
    private void CalculateForces()
    {
        dt = Time.deltaTime;

        worldPosition = _transform.position;
        transformRotation = _transform.rotation;
        velocityMagnitude = rb.velocity.magnitude;
        angularVelocityMagnitude = rb.angularVelocity.magnitude;

        //PhysicsResult physicsResult = new PhysicsResult();
        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;
        int underwaterVerts = 0;

        for (var index = 0; index < normalsCount; index++)
        {
            worldVertPos = worldPosition + (transformRotation * verts[index]);

            if (worldVertPos.y < waterLineHack)
            {
                // Splashes only on surface of water plane
                if (worldVertPos.y > waterLineHack - 0.1f)
                {
                    if (velocityMagnitude > splashVelocityThreshold ||
                        angularVelocityMagnitude > splashVelocityThreshold)
                    {
                        //print(velocityMagnitude); // Gerallt: Slow to Debug.Log
                        if (OnSplash != null)
                        {
                            OnSplash.Invoke(gameObject, worldVertPos, rb.velocity);
                        }
                    }
                }

                // forceAmount = ((transformRotation * -normals[index]) * forceScalar) * dt;
                // forcePosition = worldVertPos;
                
                forceAmount = ((-normals[index]) * forceScalar) * dt;
                forcePosition = verts[index];

                //rb.AddForceAtPosition(forceAmount, forcePosition, ForceMode.Force);
                netForce += forceAmount;
                
                //Torque is the cross product of the distance to center of mass and the force vector.
                // netTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);
                netTorque += Vector3.Cross(forcePosition, forceAmount);

                underwaterVerts++;
            }

            // HACK to remove sunken boats
            if (worldVertPos.y < waterLineHack - 10f && DestroyFallenBoats)
            {
                DestroyParentGO();
                break;
            }
        }

        if (underwaterVerts > 0 && rb != null)
        {
            // Drag for percentage underwater
            rb.drag = (underwaterVerts / (float) vertsLength) * dragScalar;
            rb.angularDrag = (underwaterVerts / (float) vertsLength) * dragScalar;

            // rb.AddForce(netForce, ForceMode.Force);
            // rb.AddTorque(netTorque, ForceMode.Force);
            
            rb.AddRelativeForce(netForce, ForceMode.Force);
            rb.AddRelativeTorque(netTorque, ForceMode.Force);
        }
    }

    private void CalculateForcesSIMD()
    {
        NativeArray<int> underwaterVertsIns = new NativeArray<int>(1, Allocator.Persistent);
        NativeArray<float3> computedForces = new NativeArray<float3>(2, Allocator.Persistent);
        
        BoatPhysicsJob physicsJob = new BoatPhysicsJob()
        {
            vertsSimd = vertsSimd,
            normalsSimd = normalsSimd,
            normalsLength = normalsCount,
            waterLineHack = waterLineHack,
            forceScalar = forceScalar,
            dt = Time.deltaTime,
            worldY = _transform.position.y,
            transformRotation = _transform.rotation,
            underwaterVertsIns = underwaterVertsIns,
            computedForces = computedForces
        };

        JobHandle physicsJobHandle = physicsJob.Schedule(); // SIMD Calculate Forces
        physicsJobHandle.Complete(); // SIMD Calculate Forces

        int underwaterVerts = physicsJob.underwaterVertsIns[0];
        
        // Update Rigidbody Physics with new computed forces.
        if (underwaterVerts > 0 && rb != null)
        {
            float3 netForceSimd = physicsJob.computedForces[0];
            float3 netTorqueSimd = physicsJob.computedForces[1];
         
            Vector3 netForce = new Vector3(netForceSimd.x, netForceSimd.y, netForceSimd.z);
            Vector3 netTorque = new Vector3(netTorqueSimd.x, netTorqueSimd.y, netTorqueSimd.z);
            
            // Drag for percentage underwater
            rb.drag = (underwaterVerts / (float) vertsLength) * dragScalar;
            rb.angularDrag = (underwaterVerts / (float) vertsLength) * dragScalar;

            rb.AddRelativeForce(netForce, ForceMode.Force);
            rb.AddRelativeTorque(netTorque, ForceMode.Force);
        }

        underwaterVertsIns.Dispose();
        computedForces.Dispose();
    }
    
    private void CalculateForcesThreaded()
    {
        worldPosition = _transform.position;
        transformRotation = _transform.rotation;
        velocityMagnitude = rb.velocity.magnitude;
        angularVelocityMagnitude = rb.angularVelocity.magnitude;

        dt = Time.deltaTime;

        underwaterVertsCount = 0;

        // Divide the work amongst other threads:
        //CalculateForcesThreadedAsync();

        // Execute any scheduled actions on the main thread.
        if (!actionsQueue.IsEmpty)
        {
            while (actionsQueue.TryDequeue(out var action))
            {
                action?.Invoke();
            }
        }
        
        while (bodyUpdatesQueue.Count > 0)
        {
            //PhysicsResult physicsResult = bodyUpdatesQueue.Take();
            bodyUpdatesQueue.TryDequeue(out PhysicsResult physicsResult);
            
            // Update the current Rigidbody with the forces calculated from other threads: 
            if (rb != null && physicsResult.UnderwaterVerts > 0)
            {
                // Append underwater verticies count to be used later for applying drag forces.
                underwaterVertsCount += physicsResult.UnderwaterVerts;

                // Apply calculated net forces to rigidbody 
                rb.AddForce(physicsResult.NetForce, ForceMode.Force);
                rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
            }
        }
        

        //HACK: once all threads finish, update drag given updated underwaterVertsCount:
        //if (task1Finished && task2Finished && task3Finished && task4Finished)
        
        if (//physicsTaskList.TrueForAll(item=> item.Complete)
            underwaterVertsCount != 0
            && rb != null
            )
        {
            // Apply drag for percentage underwater
            rb.drag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
            rb.angularDrag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;

            underwaterVertsCount = 0;
        }
    }

    private void CalculateForcesSync()
    {
        worldPosition = _transform.position;
        transformRotation = _transform.rotation;
        velocityMagnitude = rb.velocity.magnitude;
        angularVelocityMagnitude = rb.angularVelocity.magnitude;

        dt = Time.deltaTime;

        // Divide the work amongst four threads:
        CalculateForcesTaskSync();

        // Execute any scheduled actions on the main thread.
        if (!actionsQueue.IsEmpty)
        {
            while (actionsQueue.TryDequeue(out var action))
            {
                action?.Invoke();
            }
        }
    }

    public class PhysicsTask
    {
        public readonly object taskLock = new object();
        public Thread task;
        
        //Initialize semaphore, set it to BLOCK
        public ManualResetEvent sema = new ManualResetEvent(false);

        public int StartIndex;
        public int EndIndex;
        //public bool Started = false;
        //public bool Complete = false;
    }
    
    private void SetupThreads()
    {
        // Single task test:
        singleTask = new Task<PhysicsResult>(() => CalculateForces(0, normalsCount));

        // Multiple task test:

        PhysicsTask physicsTask1 = new PhysicsTask();
        PhysicsTask physicsTask2 = new PhysicsTask();
        PhysicsTask physicsTask3 = new PhysicsTask();
        PhysicsTask physicsTask4 = new PhysicsTask();

        physicsTask1.StartIndex = 0;
        physicsTask1.EndIndex = QuarterSize;
        physicsTask2.StartIndex = QuarterSize;
        physicsTask2.EndIndex = 2 * QuarterSize;
        physicsTask3.StartIndex = 2 * QuarterSize;
        physicsTask3.EndIndex = 3 * QuarterSize;
        physicsTask4.StartIndex = 3 * QuarterSize;
        physicsTask4.EndIndex = normalsCount;
        
        physicsTaskList.Add(physicsTask1);
        physicsTaskList.Add(physicsTask2);
        physicsTaskList.Add(physicsTask3);
        physicsTaskList.Add(physicsTask3);
    }

    private void CalculateForcesTaskSync()
    {
        if (singleTask.Status == TaskStatus.RanToCompletion)
        {
            // Reallocate task since C# Tasks can't be reused.
            singleTask = new Task<PhysicsResult>(() => CalculateForces(0, normalsCount));
        }

        if (singleTask.Status != TaskStatus.Running)
        {
            singleTask.RunSynchronously();

            PhysicsResult physicsResult = new PhysicsResult();
            physicsResult = singleTask.Result;

            if (rb != null)
            {
                // Drag for percentage underwater
                rb.drag = ((physicsResult.UnderwaterVerts) / (float) vertsLength) * dragScalar;
                rb.angularDrag = ((physicsResult.UnderwaterVerts) / (float) vertsLength) * dragScalar;

                // Apply calculated net forces to rigidbody 
                rb.AddForce(physicsResult.NetForce, ForceMode.Force);
                rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
            } 
        }
    }

    private void ApplyPhysics(PhysicsResult result)
    {
        if (bodyUpdatesQueue.Count > 10)
        {
            
        }
        else
        {
            bodyUpdatesQueue.Enqueue(result);
        }
        
        //bodyUpdatesQueue.Add(result);
    }

    private void CalculateForcesThreadedAsync()
    {
        for (int i = 0; i < physicsTaskList.Count; i++)
        {
            PhysicsTask physicsTask = physicsTaskList[i];

            
            Action onCompleted = () => 
            {  
                //physicsTask.Complete = true;
                //physicsTask.Started = false;
            };
            
            if (physicsTask.task == null)
            {
                physicsTask.task = new Thread(() =>
                {
                    try
                    {
                        //physicsTask.sema.Set();
                        
                        while (true)
                        {
                            Thread.Sleep(10);
                            
                            //physicsTask.Complete = false;
                        
                            PhysicsResult result = CalculateForces(physicsTask.StartIndex, physicsTask.EndIndex);
                    
                            ApplyPhysics(result);
                            
                            //physicsTask.Complete = true;
                        }
                    }
                    finally
                    {
                        onCompleted();
                    }
                });
            }

            if (physicsTask.task.ThreadState == ThreadState.Unstarted)
            {
                // physicsTask.Complete = false;
                // physicsTask.Started = true;
                    
                physicsTask.task.Start();
                //physicsTask.sema.WaitOne();
            }
        }
    }

    public class PhysicsResult
    {
        public Vector3 NetForce = Vector3.zero;
        public Vector3 NetTorque = Vector3.zero;
        public int UnderwaterVerts = 0;

        public void ClearState()
        {
            NetForce = Vector3.zero;
            NetTorque = Vector3.zero;
            UnderwaterVerts = 0;
        }
    }

    private PhysicsResult CalculateForces(int startIndex, int endIndex)
    {
        PhysicsResult physicsResult = new PhysicsResult();
        physicsResult.ClearState();

        for (var index = startIndex; index < endIndex; index++)
        {
            worldVertPos = worldPosition + TransformDirection(transformRotation, verts[index]);

            if (worldVertPos.y < waterLineHack)
            {
                // Splashes only on surface of water plane
                if (worldVertPos.y > waterLineHack - 0.1f)
                {
                    if (velocityMagnitude > splashVelocityThreshold ||
                        angularVelocityMagnitude > splashVelocityThreshold)
                    {
                        if (OnSplash != null)
                        {
                            OnSplash.Invoke(gameObject, worldVertPos, rb.velocity);
                        }
                    }
                }

                forceAmount = (TransformDirection(transformRotation, -normals[index]) * forceScalar) * dt;
                forcePosition = worldPosition + TransformDirection(transformRotation, verts[index]);

                //rb.AddForceAtPosition(forceAmount, forcePosition, ForceMode.Force);
                physicsResult.NetForce += forceAmount;

                //Torque is the cross product of the distance to center of mass and the force vector.
                physicsResult.NetTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);

                physicsResult.UnderwaterVerts++;
            }

            // HACK to remove sunken boats
            if (worldVertPos.y < waterLineHack - 10f)
            {
                if (DestroyFallenBoats)
                {
                    actionsQueue.Enqueue(() =>
                    {
                        // This code will run on the main thread

                        DestroyParentGO();
                    });
                }

                break;
            }
        }

        return physicsResult;
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