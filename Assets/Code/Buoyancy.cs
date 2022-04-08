using System;
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

    // ComputeForcesTransform()
    private bool bCalculateForcesTransform_FixedUpdate = false;
    private Vector3 acceleration = Vector3.zero;
    private Vector3 angularAcceleration = Vector3.zero;
    private Vector3 linearVelocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;
    private Vector3 netForces = Vector3.zero;
    private Vector3 netTorques = Vector3.zero;
    private float dragForce = 0;

    private Task<PhysicsResult> singleTask;
    private List<PhysicsTask> physicsTaskList = new List<PhysicsTask>();
    private Task task1;
    private Task task2;
    private Task task3;
    private Task task4;

    private int vertsLength;
    private int normalsCount;
    private int QuarterSize => normalsCount / 4;
    private Vector3[] verts;
    private Vector3[] normals;
    private Vector3 worldPosition;
    private float velocityMagnitude;
    private float angularVelocityMagnitude;
    private Vector3 normalWorldPosition;

    private float3[] vertsSIMD;
    private float3[] normalsSIMD;
    private bool bCalculateForcesSimdLateUpdate = false;
    private bool bCalculateForcesSimdParallelLateUpdate = false;
    private bool bPhysicsJobHandleParallel2LateUpdate = false;

    /// <summary>
    /// The queue of actions to execute on the main thread.
    /// </summary>
    private ConcurrentQueue<Action> actionsQueue = new ConcurrentQueue<Action>();

    /// <summary>
    /// Queue updates from other threads to the rigidbody.
    /// </summary>
    private ConcurrentQueue<PhysicsResult> bodyUpdatesQueue = new ConcurrentQueue<PhysicsResult>();
    //private BlockingCollection<PhysicsResult> bodyUpdatesQueue = new BlockingCollection<PhysicsResult>(10);


    private BoatPhysicsJob physicsJob;
    private JobHandle physicsJobHandle;
    private NativeArray<float3> vertsSimd;
    private NativeArray<float3> normalsSimd;
    private NativeArray<int> underwaterVertsIns;
    private NativeArray<float3> computedForces;

    private NativeArray<int> underwaterVertsIns2;
    private NativeArray<float3> forces;
    private NativeArray<float3> torques;
    private BoatPhysicsJobParallel physicsJobParallel;
    private JobHandle physicsJobHandleParallel;

    // One big physics job that periodically runs without being bound to Update()
    private static BoatPhysicsJobParallel2 physicsJobParallel2;
    private static JobHandle physicsJobHandleParallel2;
    private NativeArray<float3> positions; // positions of all boats 
    private NativeArray<quaternion> rotations; // rotations of all boats 
    private NativeArray<bool> isDirty; // Flags to check if there are any transform changes for all boats.
    private static bool loaded = false;
    private int boatIndex; // The index of the current boat
    private static int boatCount = 0; // The total number of boats
    private float myLastTime = 0; // Last time the fixed update ran, used to calculate custom delta time using Unity's Time.time
    private float mass; // Mass of the current boat.
    private float boatMassInverse; // Inverse mass of boat.
    private Vector3 inertia; // Inertia of the current boat.
    private Vector3 inertiaInverse; // Inverse inertia of the current boat.
    
    private void Awake()
    {
        boatCount++;
        boatIndex = boatCount - 1;
    }

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

        //SetupThreads();
        //CalculateForcesThreadedAsync();

        InitJobs();
    }

    void Update()
    {
        dt = Time.deltaTime;
        
        //CalculateForces(); // Slightly more optimised original ComputeForces
        //CalculateForcesTransform(); // Like the one above but applying forces directly to Transform trying to emulate what Unity Rigidbody does
        CalculateForcesSIMD(); // Using Unity.Mathematics optimisations to vector math and running this in a single Unity Job every frame update
        //CalculateForcesSimdParallel(); // Computing forces using Parallel Unity Job  
        //CalculateForcesSimdParallel2(); // Computing forces using Parallel Unity Job, but applying forces directly to Transform
        //CalculateForcesBetter(); // Not much different to CalculateForces()
        //CalculateForcesSync(); // CalcualteForces() but running in a synchronous C# Task and sending updates to rigidbody by a queue.
        //CalculateForcesThreaded(); // CalcualteForces() but running in a separate C# Thread and sending updates to rigidbody by a queue.
    }

    private void FixedUpdate()
    {
        dt = Time.fixedDeltaTime;

        if (bCalculateForcesTransform_FixedUpdate)
        {
            CalculateForcesTransform_FixedUpdate();
        }
    }

    private void OnDestroy()
    {
        // Dispose of NativeArrays
        vertsSimd.Dispose();
        normalsSimd.Dispose();
        underwaterVertsIns.Dispose();
        computedForces.Dispose();

        underwaterVertsIns2.Dispose();
        forces.Dispose();
        torques.Dispose();

        boatCount--;
        boatIndex--;

        if (boatCount == 0)
        {
            positions.Dispose();
            loaded = false;
        }
    }

    private void InitJobs()
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

        underwaterVertsIns = new NativeArray<int>(1, Allocator.Persistent);
        computedForces = new NativeArray<float3>(2, Allocator.Persistent);

        underwaterVertsIns2 =
            new NativeArray<int>(1 * BoatPhysicsJobParallel.NUM_JOBS, Allocator.Persistent); // More for parallel job
        forces = new NativeArray<float3>(1 * BoatPhysicsJobParallel.NUM_JOBS, Allocator.Persistent);
        torques = new NativeArray<float3>(1 * BoatPhysicsJobParallel.NUM_JOBS, Allocator.Persistent);

        if (loaded == false)
        {
            // positions = new NativeArray<float3>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);
            // rotations = new NativeArray<quaternion>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);
            // isDirty = new NativeArray<bool>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);
            //
            // // Clear flags;
            // for (int i = 0; i < BoatPhysicsJobParallel2.NumBoats; i++)
            // {
            //     isDirty[i] = false;
            // }

            // physicsJobParallel2 = new BoatPhysicsJobParallel2();
            // physicsJobParallel2.boatMass = rb.mass;
            // physicsJobParallel2.vertsSimd = vertsSimd;
            // physicsJobParallel2.normalsSimd = normalsSimd;
            // physicsJobParallel2.normalsLength = normalsCount;
            // physicsJobParallel2.vertsLength = vertsLength;
            // physicsJobParallel2.waterLineHack = waterLineHack;
            // physicsJobParallel2.forceScalar = forceScalar;
            // physicsJobParallel2.dragScalar = dragScalar;
            // physicsJobParallel2.positions = positions;
            // physicsJobParallel2.rotations = rotations;
            // physicsJobParallel2.isDirty = isDirty;
            //
            // int boats = BoatPhysicsJobParallel2.NumBoats;
            // int jobs = BoatPhysicsJobParallel2.NumJobs;
            // int boatsPerJob = (boats / jobs);
            // int jobSize = normalsCount * boatsPerJob; // Inner loop batch size
            //
            // // Schedule infinite physics job:
            // physicsJobHandleParallel2 = physicsJobParallel2.Schedule(jobs, jobSize);

            loaded = true;
        }

        positions = new NativeArray<float3>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);
        rotations = new NativeArray<quaternion>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);
        isDirty = new NativeArray<bool>(BoatPhysicsJobParallel2.NumBoats, Allocator.Persistent);

        // Clear flags;
        for (int i = 0; i < BoatPhysicsJobParallel2.NumBoats; i++)
        {
            isDirty[i] = false;
        }
        
        mass = rb.mass;
        boatMassInverse = 1.0f / mass;
        //inertia = rb.inertiaTensorRotation * rb.inertiaTensor; // Is the inertia tensor already rotated? and do I need to update this as it rotates every FixedUpdate()
        inertia = rb.inertiaTensor;
        inertiaInverse = new Vector3(1.0f / inertia.x, 1.0f / inertia.y, 1.0f / inertia.z);
    }

    private void CalculateForcesSimdParallel2()
    {
        if (boatIndex >= BoatPhysicsJobParallel2.NumBoats) //HACK
            return;
        
        bPhysicsJobHandleParallel2LateUpdate = true;
        physicsJobParallel2.boatMass = rb.mass;
        physicsJobParallel2.vertsSimd = vertsSimd;
        physicsJobParallel2.normalsSimd = normalsSimd;
        physicsJobParallel2.normalsLength = normalsCount;
        physicsJobParallel2.vertsLength = vertsLength;
        physicsJobParallel2.waterLineHack = waterLineHack;
        physicsJobParallel2.forceScalar = forceScalar;
        physicsJobParallel2.dragScalar = dragScalar;
        physicsJobParallel2.positions = positions;
        physicsJobParallel2.rotations = rotations;
        physicsJobParallel2.isDirty = isDirty;
            
        int boats = BoatPhysicsJobParallel2.NumBoats;
        int jobs = BoatPhysicsJobParallel2.NumJobs;
        int boatsPerJob = (int)(boats / (float)jobs);
        int jobSize = normalsCount * boatsPerJob; // Inner loop batch size
            
        // Schedule infinite physics job:
        physicsJobHandleParallel2 = physicsJobParallel2.Schedule(jobs, jobSize);

        if (physicsJobParallel2.isDirty[boatIndex])
        {
            this._transform.position = physicsJobParallel2.positions[boatIndex];
            this._transform.rotation = physicsJobParallel2.rotations[boatIndex];
        
            physicsJobParallel2.isDirty[boatIndex] = false;
        }
    }
    
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

    private void CalculateForcesTransform()
    {
        // Precondition: need to have a dt
        bCalculateForcesTransform_FixedUpdate = true; // Run a FixedUpdate to apply the forces
        
        worldPosition = _transform.position;
        transformRotation = _transform.rotation;

        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;
        int underwaterVerts = 0;

        for (var index = 0; index < normalsCount; index++)
        {
            worldVertPos = worldPosition + (transformRotation * verts[index]);

            if (worldVertPos.y < waterLineHack)
            {
                forceAmount = ((-normals[index]) * forceScalar) * dt;
                forcePosition = verts[index];

                netForce += forceAmount;

                //Torque is the cross product of the distance to center of mass and the force vector.
                netTorque += Vector3.Cross(forcePosition, forceAmount);

                underwaterVerts++;
            }
        }


        // Defer applying these until FixedUpdate
        netForces = netForce;
        netTorques = netTorque;
        dragForce = 0;

        if (underwaterVerts > 0)
        {
            // Drag for percentage underwater
            float dragCoefficient = ((underwaterVerts / (float) vertsLength) * this.dragScalar);
            dragForce = dragCoefficient;
        }
    }

    private void CalculateForcesTransform_FixedUpdate()
    {
        // Custom delta time test
        if (myLastTime != 0)
        {
            //dt = Time.time - myLastTime;

            //CalculateForcesTransform();

            rb.isKinematic = true;
            if (rb.useGravity)
            {
                // Apply gravity force
                Vector3 gravityForce = Physics.gravity * (mass * mass);
                netForces += gravityForce;
            }

            // Apply forces to boat transform.
            acceleration = ((netForces) * boatMassInverse); // F=ma, a=F/m
            //angularAcceleration = netTorques * boatMassInverse; // F=ma, a=F/m
            angularAcceleration = Vector3.Scale(netTorques, inertiaInverse); // F=ma, a=F/m

            //dragCoefficient = 0;
            float velocityDampening = 1.0f - (dragForce * dt);
            if (velocityDampening < 0.0f) velocityDampening = 0.0f;
            //float velocityDampening = Mathf.Clamp(1.0f - (dt * (dragForce + 10)), 0.0f, 10000.0f);
            
            Vector3 velocityChange = acceleration * dt; // linear velocity
            Vector3 angularVelocityChange = angularAcceleration * dt; // rotational velocity

            // Apply drag dampening force
            linearVelocity += velocityChange;
            linearVelocity *= velocityDampening;
            angularVelocity += angularVelocityChange;
            angularVelocity *= velocityDampening;

            // Integrate to find new position and rotation
            Vector3 positionDelta = linearVelocity * dt;
            Quaternion rotationDelta = Quaternion.Euler(angularVelocity * dt);
            
            _transform.position += positionDelta;
            _transform.rotation *= rotationDelta;
            
            // Update Forward Vector based on new velocity
            //_transform.LookAt(_transform.position + linearVelocity);
            //_transform.forward = linearVelocity.normalized;
            _transform.forward = Vector3.Slerp(_transform.forward, linearVelocity.normalized, dt);
            
            // Apply drag dampening force
            // linearVelocity += velocityChange;
            // linearVelocity *= velocityDampening;
            // angularVelocity += angularVelocityChange;
            // angularVelocity *= velocityDampening;
        }
        else
        {
            linearVelocity = rb.velocity;
            angularVelocity = rb.angularVelocity;
        }

        myLastTime = Time.time;
    }
    
    private void CalculateForcesSIMD()
    {
        bCalculateForcesSimdLateUpdate = true;

        physicsJob.vertsSimd = vertsSimd;
        physicsJob.normalsSimd = normalsSimd;
        physicsJob.normalsLength = normalsCount;
        physicsJob.waterLineHack = waterLineHack;
        physicsJob.forceScalar = forceScalar;
        physicsJob.dt = Time.deltaTime;
        physicsJob.worldY = _transform.position.y;
        physicsJob.transformRotation = _transform.rotation;
        physicsJob.underwaterVertsIns = underwaterVertsIns;
        physicsJob.computedForces = computedForces;

        physicsJobHandle = physicsJob.Schedule(); // SIMD Calculate Forces
    }

    private void CalculateForcesSimdParallel()
    {
        bCalculateForcesSimdParallelLateUpdate = true;

        physicsJobParallel.vertsSimd = vertsSimd;
        physicsJobParallel.normalsSimd = normalsSimd;
        physicsJobParallel.normalsLength = normalsCount;
        physicsJobParallel.waterLineHack = waterLineHack;
        physicsJobParallel.forceScalar = forceScalar;
        physicsJobParallel.dt = Time.deltaTime;
        physicsJobParallel.worldY = _transform.position.y;
        physicsJobParallel.transformRotation = _transform.rotation;
        physicsJobParallel.underwaterVertsIns = underwaterVertsIns2;
        physicsJobParallel.forces = forces;
        physicsJobParallel.torques = torques;

        // SIMD Calculate Forces batch task
        int jobs = BoatPhysicsJobParallel.NUM_JOBS;
        int jobSize = normalsCount / jobs;
        physicsJobHandleParallel = physicsJobParallel.Schedule(jobs, jobSize);
    }

    private void LateUpdate()
    {
        if (bPhysicsJobHandleParallel2LateUpdate && boatIndex < BoatPhysicsJobParallel2.NumBoats)
        {
            physicsJobHandleParallel2.Complete();
            if (physicsJobParallel2.isDirty[boatIndex])
            {
                this._transform.position = physicsJobParallel2.positions[boatIndex];
                this._transform.rotation = physicsJobParallel2.rotations[boatIndex];

                physicsJobParallel2.isDirty[boatIndex] = false;
            }
        }

        if (bCalculateForcesSimdLateUpdate)
        {
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
                float dragCoefficient = ((underwaterVerts / (float) vertsLength) * dragScalar);

                rb.drag = dragCoefficient;
                rb.angularDrag = dragCoefficient;

                rb.AddRelativeForce(netForce, ForceMode.Force);
                rb.AddRelativeTorque(netTorque, ForceMode.Force);
            }
        }
        else if (bCalculateForcesSimdParallelLateUpdate)
        {
            physicsJobHandleParallel.Complete(); // SIMD Calculate Forces batch task

            int underwaterVerts = 0;
            Vector3 netForce = Vector3.zero;
            Vector3 netTorque = Vector3.zero;

            for (int jobId = 0; jobId < BoatPhysicsJobParallel.NUM_JOBS; jobId++)
            {
                int underVerts = physicsJobParallel.underwaterVertsIns[jobId];

                underwaterVerts += underVerts;

                if (underVerts > 0)
                {
                    float3 forceSimd = physicsJobParallel.forces[jobId];
                    float3 torqueSimd = physicsJobParallel.torques[jobId];

                    netForce += new Vector3(forceSimd.x, forceSimd.y, forceSimd.z);
                    netTorque += new Vector3(torqueSimd.x, torqueSimd.y, torqueSimd.z);
                }
            }

            // Update Rigidbody Physics with new computed forces.
            if (underwaterVerts > 0 && rb != null)
            {
                // Drag for percentage underwater
                float dragCoefficient = ((underwaterVerts / (float) vertsLength) * dragScalar);

                rb.drag = dragCoefficient;
                rb.angularDrag = dragCoefficient;

                rb.AddRelativeForce(netForce, ForceMode.Force);
                rb.AddRelativeTorque(netTorque, ForceMode.Force);
            }
        }
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

        if ( //physicsTaskList.TrueForAll(item=> item.Complete)
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