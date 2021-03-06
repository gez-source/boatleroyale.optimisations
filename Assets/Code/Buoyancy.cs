using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using Gerallt;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityStandardAssets.Water;
using static Unity.Mathematics.math;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine.Jobs;
using UnityEngine.Serialization;
using float3 = Unity.Mathematics.float3;
using float4 = Unity.Mathematics.float4;
using quaternion = Unity.Mathematics.quaternion;

// Cams mostly hack buoyancy
public class Buoyancy : MonoBehaviour
{
    public float splashVelocityThreshold;
    public float forceScalar;
    public float areaScalar;
    public float cullAngle;
    public bool dontCullTopNormals;
    public float waterLineHack; // HACK
    public float waterDepthScalar;
    public float dragScalar;
    public Vector3 aerodynamicDrag;

    public static event Action<GameObject, Vector3, Vector3> OnSplash;
    public static event Action<GameObject> OnDestroyed;

    Vector3 worldVertPos;

    // Gerallt
    public Algorithm algorithmType = Algorithm.OriginalCalculateForces;
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
    private Vector3 centerOfMass; // Center of mass of boat mesh.
    private float totalSurfaceArea; // Total surface area of boat mesh.
    public float maxSurfaceArea;
    
    private Task<PhysicsResult> singleTask;
    private List<PhysicsTask> physicsTaskList = new List<PhysicsTask>();
    private Task task1;
    private Task task2;
    private Task task3;
    private Task task4;

    public int vertsLength;
    public int normalsCount;
    private int QuarterSize => normalsCount / 4;
    private Vector3[] verts;
    private Vector3[] normals;
    private int[] faces = null;
    private float[] polygonAreas = null;
    private Vector3 worldPosition;
    private float velocityMagnitude;
    private float angularVelocityMagnitude;
    private Vector3 normalWorldPosition;

    private float3[] vertsSIMD;
    private float3[] normalsSIMD;
    private bool bCalculateForcesSimdLateUpdate = false;
    private bool bCalculateForcesSimdSurfacesLateUpdate = false;
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
    public NativeArray<float3> vertsSimd;
    public NativeArray<float3> normalsSimd;
    private NativeArray<int> underwaterVertsIns;
    private NativeArray<float3> computedForces;

    private BoatPhysicsJobSurfaces physicsJobSurfaces;
    private JobHandle physicsJobHandleSurfaces;
    public NativeArray<int> facesSimd;
    public NativeArray<float> polygonAreasSimd;
    
    private NativeArray<int> underwaterVertsIns2;
    private NativeArray<float3> forces;
    private NativeArray<float3> torques;
    private BoatPhysicsJobParallel physicsJobParallel;
    private JobHandle physicsJobHandleParallel;

    // One big physics job that periodically runs without being bound to Update()
    // private static bool isRunningParallelJob = false;
    // public static BoatPhysicsJobSurfacesParallel physicsJobSurfacesParallel; // NEW
    // public static JobHandle physicsJobHandleSurfacesParallel;
    // public static bool bCalculateForcesSimdParallel2LateUpdate;
    // private static TransformAccessArray? transformAccessArray; 
    
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
    
    public enum Algorithm
    {
        OriginalCalculateForces,
        CalculateForcesOnSurface,
        CalculateForcesOnSurfaceSIMD,
        CalculateForcesOnSurfaceSIMDParallel,
        CalculateForcesTransform,
        CalculateForcesSIMD,
        CalculateForcesSimdParallel,
        CalculateForcesSimdParallel2,
        CalculateForcesSync,
        CalculateForcesThreaded,
        NotSet
    }
    
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

        // Find mesh center of mass and mesh surface area which is used by custom physics.
        faces = mesh.triangles;
        TraverseMesh();

        //SetupThreads();
        //CalculateForcesThreadedAsync();

        InitJobs();
    }
    
    public void TraverseMesh()
    {
        totalSurfaceArea = 0f;
        maxSurfaceArea = float.MinValue;
        centerOfMass = Vector3.zero;

        polygonAreas = new float[faces.Length / 3];
        
        for (var faceIndex = 0; faceIndex <= faces.Length - 3; faceIndex += 3)
        {
            var index0 = faces[faceIndex];
            var index1 = faces[faceIndex + 1];
            var index2 = faces[faceIndex + 2];
            
            Vector3 v1 = verts[index0];
            Vector3 v2 = verts[index1];
            Vector3 v3 = verts[index2];

            Vector3 edge1 = v2 - v1;
            Vector3 edge2 = v3 - v1;

            // Calculate entire surface area of boat and center of mass while we are at it.
            float triangleArea = Vector3.Cross(edge1, edge2).sqrMagnitude / 2.0f;

            polygonAreas[faceIndex / 3] = triangleArea; // Store precalculated polygon area.
            
            if (triangleArea > maxSurfaceArea)
            {
                maxSurfaceArea = triangleArea;
            }
            
            totalSurfaceArea += triangleArea;
            centerOfMass += (v1 + v2 + v3);

            normals[index0] = normals[index0].normalized;
            normals[index1] = normals[index1].normalized;
            normals[index2] = normals[index2].normalized;
        }

        if (vertsLength > 0)
        {
            centerOfMass /= vertsLength;
        }
    }

    void Update()
    {
        dt = Time.deltaTime;

        switch (algorithmType)
        {
            case Algorithm.OriginalCalculateForces:
                CalculateForces(); // Slightly more optimised original ComputeForces
                break;
            case Algorithm.CalculateForcesOnSurface:
                CalculateForcesOnSurface(); // Calculates forces proportional to the boats surface area 
                break;
            case Algorithm.CalculateForcesOnSurfaceSIMD:
                CalculateForcesOnSurfaceSIMD(); // SIMD optimised version of calculating forces proportional to the boats surface area 
                break;
            case Algorithm.CalculateForcesOnSurfaceSIMDParallel:
                //CalculateForcesOnSurfaceSIMDParallel(); // Called from Spawner's Update()
                break;
            case Algorithm.CalculateForcesTransform:
                CalculateForcesTransform(); // Like the one above but applying forces directly to Transform trying to emulate what Unity Rigidbody does
                break;
            case Algorithm.CalculateForcesSIMD:
                CalculateForcesSIMD(); // Using Unity.Mathematics optimisations to vector math and running this in a single Unity Job every frame update
                break;
            case Algorithm.CalculateForcesSimdParallel:
                CalculateForcesSimdParallel(); // Computing forces using Parallel Unity Job  
                break;
            case Algorithm.CalculateForcesSimdParallel2:
                CalculateForcesSimdParallel2(); // Computing forces using Parallel Unity Job, but applying forces directly to Transform
                break;
            case Algorithm.CalculateForcesSync:
                CalculateForcesSync(); // CalcualteForces() but running in a synchronous C# Task and sending updates to rigidbody by a queue.
                break;
            case Algorithm.CalculateForcesThreaded:
                CalculateForcesThreaded(); // CalcualteForces() but running in a separate C# Thread and sending updates to rigidbody by a queue.
                break;
        }
        //CalculateForcesBetter(); // Not much different to CalculateForces()
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
        
        // if (boatCount <= 0)
        // {
        //     positions.Dispose();
        //     rotations.Dispose();
        //     isDirty.Dispose();
        //     loaded = false;
        // }
        
        positions.Dispose();
        rotations.Dispose();
        isDirty.Dispose();

        polygonAreasSimd.Dispose();
        facesSimd.Dispose();
    }

    private void InitJobs()
    {
        // Convert Unity Vectors to Unity.Mathematics SIMD types:
        int normIdx;
        int vert;
        int poly;
        int faceIndex;

        vertsSimd = new NativeArray<float3>(vertsLength, Allocator.Persistent);
        normalsSimd = new NativeArray<float3>(normalsCount, Allocator.Persistent);
        polygonAreasSimd = new NativeArray<float>(polygonAreas.Length, Allocator.Persistent);
        facesSimd = new NativeArray<int>(faces.Length, Allocator.Persistent);
        
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

        for (poly = 0; poly < polygonAreas.Length; poly++)
        {
            polygonAreasSimd[poly] = polygonAreas[poly];
        }
        
        for (faceIndex = 0; faceIndex < faces.Length; faceIndex++)
        {
            facesSimd[faceIndex] = faces[faceIndex];
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
            float dragCoefficient =  (underwaterVerts / (float) vertsLength) * dragScalar;
            rb.drag = dragCoefficient;
            rb.angularDrag = dragCoefficient;

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
                netTorque += Vector3.Cross(forcePosition, forceAmount);
                //netTorque += Vector3.Cross(forcePosition - centerOfMass, forceAmount);

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
            float dragCoefficient = ((underwaterVerts / (float) vertsLength) * this.dragScalar);
            
            rb.drag = dragCoefficient;
            rb.angularDrag = dragCoefficient;

            // rb.AddForce(netForce, ForceMode.Force);
            // rb.AddTorque(netTorque, ForceMode.Force);

            rb.AddRelativeForce(netForce, ForceMode.Force);
            rb.AddRelativeTorque(netTorque, ForceMode.Force);
        }
    }

    bool TriangleIntersectsWaterline(Vector3 vw0, Vector3 vw1, Vector3 vw2, Vector3 n0, Vector3 n1, Vector3 n2, float waterLine)
    {
        // float rayDistance = 0.01f;
        //
        // bool result = Physics.Raycast(vw0, n0, rayDistance)
        //               || Physics.Raycast(vw1, n1, rayDistance)
        //               || Physics.Raycast(vw2, n2, rayDistance);
        // return result;
        return (vw0.y < waterLine) || (vw1.y < waterLine) || (vw2.y < waterLine);
    }
    
    private void CalculateForcesOnSurface()
    {
        dt = Time.deltaTime;
        
        // Precondition: need to have a dt
        bCalculateForcesTransform_FixedUpdate = false;
        
        worldPosition = _transform.position;
        transformRotation = _transform.rotation;

        float HACK = 4.0f; // HACK: had to double force before boats would float. If you are just using the Unity Rigidbody don't use the hack
        HACK = 1.0f; // Uncomment if using Unity's Rigidbody
        //HACK = 1000;
        
        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;
        int underwaterVerts = 0;
        float invMaxSurfaceArea = 1.0f / maxSurfaceArea;
        float oneThird = 1.0f / 3.0f;
        
        // Testing new surface area force contributions
        for (var faceIndex = 0; faceIndex <= faces.Length - 3; faceIndex += 3) // Face is always a triangle.
        {
            var index0 = faces[faceIndex];
            var index1 = faces[faceIndex + 1];
            var index2 = faces[faceIndex + 2];
            
            if (index0 >= vertsLength)
                continue;
            
            Vector3 v1 = verts[index0];
            Vector3 v2 = verts[index1];
            Vector3 v3 = verts[index2];

            Vector3 vw0 = worldPosition + (transformRotation * v1);
            Vector3 vw1 = worldPosition + (transformRotation * v2);
            Vector3 vw2 = worldPosition + (transformRotation * v3);
            
            Vector3 n0 = normals[index0];
            Vector3 n1 = normals[index1];
            Vector3 n2 = normals[index2];
            
            if (TriangleIntersectsWaterline(vw0, vw1, vw2, n0, n1, n2, waterLineHack))
            { 
                // Take into account the contribution of the surface area and each vertex
                float triangleArea = polygonAreas[(int)(faceIndex * oneThird)]; // Use precalculated polygon area to save time. 
                float areaRatio = triangleArea * invMaxSurfaceArea; // float areaRatio = triangleArea / maxSurfaceArea;
                float forceMagnitude = ((forceScalar * HACK) * (areaRatio * areaScalar))  * dt;

                for (var vertIndex = 0; vertIndex < 3; vertIndex++)
                {
                    var index = faces[faceIndex + vertIndex];
                    Vector3 vertPos = verts[index];
                    Vector3 vertWorldPos = worldPosition + (transformRotation * vertPos);
                    float waterDepth = (1.0f + waterLineHack) - vertWorldPos.y;
                    
                    if (vertWorldPos.y < waterLineHack)
                    {
                        Vector3 normal = normals[index];
                        
                        if (dontCullTopNormals || Vector3.Dot((transformRotation * normal), Vector3.down) > cullAngle)
                        //if (dontCullTopNormals || Vector3.Dot((transformRotation * normal).normalized, Vector3.down) > cullAngle) // Filter out parts of the boat that aren't the hull.
                        //if (dontCullTopNormals || Vector3.Dot(normal, Vector3.down) > cullAngle) 
                        {
                            //forceAmount = (transformRotation * -normal).normalized * forceScalar * dt;
                            //forceAmount = (transformRotation * (-normal)).normalized * (forceMagnitude * (waterDepth * waterDepthScalar)) ;
                            forceAmount = (transformRotation * (-normal)) * (forceMagnitude * (waterDepth * waterDepthScalar)) ;
                            
                            forcePosition = vertWorldPos;
                            //forcePosition = vertPos;
                            
                            netForce += forceAmount;
                            //netTorque += Vector3.Cross(forcePosition - centerOfMass, forceAmount);
                            netTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);
                            // Torque is the cross product of the distance to center of mass and the force vector.
                        
                            underwaterVerts++;
                        }
                    }
                }
            }
        }
        
        netForces = netForce;
        netTorques = netTorque;
        dragForce = 0;

        // Apply gravity force internally
        Vector3 gravityForce = Physics.gravity;
        rb.useGravity = false;

        if (underwaterVerts > 0 && rb != null)
        {
            // Drag for percentage underwater
            float dragCoefficient = ((underwaterVerts / (float) vertsLength) * this.dragScalar);
            dragForce = dragCoefficient;
        }

        // TESTING
        rb.drag = dragForce;
        rb.angularDrag = dragForce;
        rb.AddForce(netForces, ForceMode.Force);
        rb.AddForce(gravityForce, ForceMode.Acceleration);
        rb.AddTorque(netTorques, ForceMode.Force);
        
        netForces = Vector3.zero;
        netTorques = Vector3.zero;
    }
    
    private void CalculateForcesOnSurfaceSIMD()
    {
        bCalculateForcesSimdSurfacesLateUpdate = true;

        physicsJobSurfaces.vertsSimd = vertsSimd;
        physicsJobSurfaces.normalsSimd = normalsSimd;
        physicsJobSurfaces.normalsLength = normalsCount;
        physicsJobSurfaces.vertsLength = vertsLength;
        physicsJobSurfaces.polygonAreasSimd = polygonAreasSimd;
        physicsJobSurfaces.facesSimd = facesSimd;
        physicsJobSurfaces.maxSurfaceArea = maxSurfaceArea;
        physicsJobSurfaces.waterLineHack = waterLineHack;
        physicsJobSurfaces.forceScalar = forceScalar;
        physicsJobSurfaces.areaScalar = areaScalar;
        physicsJobSurfaces.waterDepthScalar = waterDepthScalar;
        physicsJobSurfaces.cullAngle = cullAngle;
        physicsJobSurfaces.dontCullTopNormals = dontCullTopNormals;
        physicsJobSurfaces.dt = Time.deltaTime;
        physicsJobSurfaces.worldPosition = _transform.position;
        physicsJobSurfaces.transformRotation = _transform.rotation;
        physicsJobSurfaces.underwaterVertsIns = underwaterVertsIns;
        physicsJobSurfaces.computedForces = computedForces;

        physicsJobHandleSurfaces = physicsJobSurfaces.Schedule(); // SIMD Calculate Forces on Surfaces
    }

    private void CalculateForcesTransform()
    {
        dt = Time.deltaTime;
        
        // Precondition: need to have a dt
        bCalculateForcesTransform_FixedUpdate = true; // Run a FixedUpdate to apply the forces
        
        worldPosition = _transform.position;
        transformRotation = _transform.rotation;

        float HACK = 4.0f; // HACK: had to double force before boats would float. If you are just using the Unity Rigidbody don't use the hack
        HACK = 1.0f; // Uncomment if using Unity's Rigidbody
        
        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;
        int underwaterVerts = 0;

        for (var index = 0; index < normalsCount; index++)
        {
            worldVertPos = worldPosition + (transformRotation * verts[index]);
        
            if (worldVertPos.y < waterLineHack)
            {
                forceAmount = ((-normals[index]) * forceScalar * HACK) * dt;
                forcePosition = verts[index];
        
                netForce += forceAmount;
        
                //Torque is the cross product of the distance to center of mass and the force vector.
                netTorque += Vector3.Cross(forcePosition, forceAmount);
                //netTorque += Vector3.Cross(forcePosition - centerOfMass, forceAmount);
                
                underwaterVerts++;
            }
        }
        
        // Defer applying these until FixedUpdate
        netForces = netForce;
        netTorques = netTorque;
        dragForce = 0;

        // Apply gravity force internally
        //Vector3 gravityForce = Physics.gravity;
        rb.useGravity = false;
        
        // if (rb.isKinematic == false)
        // {
        //     rb.velocity = Vector3.zero;
        //     //rb.angularVelocity = Vector3.zero;
        //     rb.drag = 0;
        //     //rb.angularDrag = 0;
        // }
        
        if (underwaterVerts > 0 && rb != null)
        {
            // Drag for percentage underwater
            float dragCoefficient = ((underwaterVerts / (float) vertsLength) * this.dragScalar);
            dragForce = dragCoefficient;
        }
        
        // TESTING
        // rb.drag = dragForce;
        // rb.angularDrag = dragForce;
        // rb.AddRelativeForce(netForces, ForceMode.Force);
        // rb.AddForce(gravityForce, ForceMode.Acceleration);
        // rb.AddRelativeTorque(netTorques, ForceMode.Force);
        //
        // netForces = Vector3.zero;
        // netTorques = Vector3.zero;
    }

    private void CalculateForcesTransform_FixedUpdate()
    {
        // Custom delta time test
        if (myLastTime != 0)
        {
            //dt = Time.time - myLastTime;

            // Apply gravity force
            //Vector3 gravityForce = _transform.worldToLocalMatrix * Physics.gravity;
            Vector3 gravityForce = Physics.gravity;
            
            // Apply forces to boat transform.
            mass = rb.mass;
            float invMass = (1.0f / mass);
            acceleration = netForces * invMass; // F=ma, a=F/m
            angularAcceleration = netTorques * invMass; // F=ma, a=F/m
            Vector3 inertiaRotated = rb.inertiaTensor * mass; //HACK: combining mass and inertia
            Vector3 inertiaRotInv = new Vector3(1.0f / inertiaRotated.x, 1.0f / inertiaRotated.y, 1.0f / inertiaRotated.z);
            //angularAcceleration = Vector3.Scale(netTorques,  inertiaRotInv); // F=ma, a=F/m, Apply inertia and mass

            // Apply acceleration due to gravity
            acceleration += gravityForce;

            // Integrate to find new velocity
            Vector3 velocityChange = acceleration * dt; // linear velocity
            Vector3 angularVelocityChange = angularAcceleration * dt; // rotational velocity
            
            linearVelocity += velocityChange;
            angularVelocity += angularVelocityChange;



            Vector3 alignmentTorque = Vector3.Cross(linearVelocity.normalized, _transform.forward.normalized); // Align to new direction
            Vector3 alignmentVelocityChange = (alignmentTorque * invMass) * dt;
            angularVelocity += alignmentVelocityChange;

            // v = v - k * v // Linear drag
            linearVelocity -= Vector3.Scale(aerodynamicDrag, linearVelocity);
            angularVelocity -= Vector3.Scale(aerodynamicDrag, angularVelocity);
            
            // v = v - k * v - m * v2 // Quadratic drag
            //linearVelocity = linearVelocity - (aerodynamicDrag * linearVelocity) - (mass * Vector3.Scale(linearVelocity, linearVelocity));
            //angularVelocity = angularVelocity - (aerodynamicDrag * angularVelocity) - (Vector3.Scale(inertiaRotated, Vector3.Scale(angularVelocity, angularVelocity)));
            //angularVelocity = angularVelocity - (aerodynamicDrag * angularVelocity) - (mass * Vector3.Scale(angularVelocity, angularVelocity));
            //angularVelocity = angularVelocity - (aerodynamicDrag * angularVelocity);
            
            // Apply drag dampening force
            float velocityDampening = 1.0f - (dragForce * dt);
            if (velocityDampening < 0.0f) velocityDampening = 0.0f;
            //float velocityDampening = Mathf.Clamp(1.0f - (dt * (dragForce)), 0.0f, 10000.0f);
            linearVelocity *= velocityDampening;
            angularVelocity *= velocityDampening; // Apply same drag value to rotation.
            
            // Integrate to find new position
            Vector3 positionDelta = linearVelocity * dt;
            _transform.position += positionDelta;
            
            // Integrate to find new rotation
            // Quaternion rotationDelta = Quaternion.Euler(angularVelocity * dt); // This is simple but it doesn't work
            // //Quaternion rotationDelta = Quaternion.LookRotation(angularVelocity * dt);
            // rotationDelta.Normalize();
            // _transform.rotation *= rotationDelta;

            Quaternion r = _transform.rotation;
            Quaternion w = new Quaternion(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0);
            Quaternion newRotation = Add(Scale(w, dt * 0.5f) * r, r);
            newRotation.Normalize();
            _transform.rotation = newRotation;
            
            // Update Forward Vector based on new velocity (Align to new direction)
            //_transform.forward = _transform.position + Vector3.Slerp(_transform.forward.normalized, linearVelocity.normalized, dt);
            //_transform.forward += Vector3.Slerp(_transform.forward.normalized, linearVelocity.normalized, dt);

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
    public static Quaternion Scale(Quaternion q, float scale)
    {
        return new Quaternion(q.x * scale, q.y * scale, q.z * scale, q.w * scale);
    }

    public static Quaternion Add(Quaternion left, Quaternion right)
    {
        return new Quaternion(left.x + right.x, left.y + right.y, left.z + right.z, left.w + right.w);;
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
        else if (bCalculateForcesSimdSurfacesLateUpdate)
        {
            physicsJobHandleSurfaces.Complete(); // SIMD Calculate Forces

            int underwaterVerts = physicsJobSurfaces.underwaterVertsIns[0];

            // Update Rigidbody Physics with new computed forces.
            if (underwaterVerts > 0 && rb != null)
            {
                float3 netForceSimd = physicsJobSurfaces.computedForces[0];
                float3 netTorqueSimd = physicsJobSurfaces.computedForces[1];

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
                physicsResult.NetTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount); // TODO: Probably should use centerOfMass

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