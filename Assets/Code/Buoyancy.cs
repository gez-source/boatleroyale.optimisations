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

    //public int underwaterVerts;
    public float dragScalar;

    public static event Action<GameObject, Vector3, Vector3> OnSplash;
    public static event Action<GameObject> OnDestroyed;

    Vector3 worldVertPos;

    // Gerallt
    private Rigidbody rb;
    private MeshFilter meshFilter;
    private Transform _transform;
    private Vector3 forceAmount;
    private Vector3 forcePosition;
    private float dt;
    private Quaternion transformRotation;
    private int underwaterVertsCount;

    private static bool loaded = false;
    private static int vertsLength;
    private static int normalsCount;
    private static Vector3[] verts;
    private static Vector3[] normals;
    private static Mesh mesh;
    private static Vector3 worldPosition;
    private static float velocityMagnitude;
    private static float angularVelocityMagnitude;
    private static Vector3 normalWorldPosition;

    /// <summary>
    /// The queue of actions to execute on the main thread.
    /// </summary>
    private static ConcurrentQueue<Action> actionsQueue = new ConcurrentQueue<Action>();
    
    /// <summary>
    /// Queue updates from other threads to the rigidbody.
    /// </summary>
    private ConcurrentQueue<PhysicsResult> bodyUpdatesQueue = new ConcurrentQueue<PhysicsResult>();

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
        CalculateForces();
        //CalculateForcesSync();
        //CalculateForcesThreaded();
    }

    private void FixedUpdate()
    {
        //CalculateForces();
    }

    private void CalculateForces()
    {
        //underwaterVerts = 0;

        dt = Time.deltaTime;

        worldPosition = _transform.position;
        transformRotation = _transform.rotation;
        velocityMagnitude = rb.velocity.magnitude;
        angularVelocityMagnitude = rb.angularVelocity.magnitude;

        PhysicsResult physicsResult = new PhysicsResult();
        physicsResult.NetForce = Vector3.zero;
        physicsResult.NetTorque = Vector3.zero;
        
        for (var index = 0; index < normalsCount; index++)
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
                        //print(velocityMagnitude); // Gerallt: Slow to Debug.Log
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
                DestroyParentGO();
                break;
            }
        }

        // Drag for percentage underwater
        rb.drag = (physicsResult.UnderwaterVerts / (float) vertsLength) * dragScalar;
        rb.angularDrag = (physicsResult.UnderwaterVerts / (float) vertsLength) * dragScalar;

        rb.AddForce(physicsResult.NetForce, ForceMode.Force);
        rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
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
        CalculateForcesThreadedAsync();

        // Execute any scheduled actions on the main thread.
        if (!actionsQueue.IsEmpty)
        {
            while (actionsQueue.TryDequeue(out var action))
            {
                action?.Invoke();
            }
        }

        if (!bodyUpdatesQueue.IsEmpty)
        {
            while (bodyUpdatesQueue.TryDequeue(out var physicsResult))
            {
                // Update the current Rigidbody with the forces calculated from other threads: 
                if (rb != null)
                {
                    underwaterVertsCount += physicsResult.UnderwaterVerts;
                    //underwaterVertsCount = physicsResult.UnderwaterVerts;

                    // // Drag for percentage underwater
                    // rb.drag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
                    // rb.angularDrag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;

                    
                    // Apply calculated net forces to rigidbody 
                    rb.AddForce(physicsResult.NetForce, ForceMode.Force);
                    rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
                }
            }
        }

        //HACK: once all threads finish, update drag given updated underwaterVertsCount:
        if (task1Finished && task2Finished && task3Finished && task4Finished)
        {
            // Drag for percentage underwater
            rb.drag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
            rb.angularDrag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
            
            task1Finished = false;
            task2Finished = false;
            task3Finished = false;
            task4Finished = false;

            underwaterVertsCount = 0;
        }
    }

    private bool task1Finished = false;
    private bool task2Finished = false;
    private bool task3Finished = false;
    private bool task4Finished = false;
    
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

    private void CalculateForcesTaskSync()
    {
        var task1 = new Task<PhysicsResult>(() => CalculateForces(0, normalsCount));

        task1.RunSynchronously();


        PhysicsResult physicsResult = task1.Result;

        if (rb != null)
        {
            int underwaterVertsCount = physicsResult.UnderwaterVerts;

            // Drag for percentage underwater
            rb.drag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
            rb.angularDrag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;

            // Apply calculated net forces to rigidbody 
            rb.AddForce(physicsResult.NetForce, ForceMode.Force);
            rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
        }
    }

    private void CalculateForcesThreadedAsync()
    {
        int quarter = normalsCount / 4;

        var task1 = Task.Run(() => CalculateForces(0, quarter))
            .ContinueWith(task =>
        {
            PhysicsResult physicsResult = task.Result;
            
            bodyUpdatesQueue.Enqueue(physicsResult);

            task1Finished = true;
        });
        var task2 = Task.Run(() => CalculateForces(quarter, 2 * quarter))
            .ContinueWith(task =>
            {
                PhysicsResult physicsResult = task.Result;
                
                bodyUpdatesQueue.Enqueue(physicsResult);
                
                task2Finished = true;
            });
        var task3 = Task.Run(() => CalculateForces(2 * quarter,  3 * quarter))
            .ContinueWith(task =>
        {
            PhysicsResult physicsResult = task.Result;

            bodyUpdatesQueue.Enqueue(physicsResult);
            
            task3Finished = true;
        });
        var task4 = Task.Run(() => CalculateForces(3 * quarter, normalsCount))
            .ContinueWith(task =>
        {
            PhysicsResult physicsResult = task.Result;
            
            bodyUpdatesQueue.Enqueue(physicsResult);
            
            task4Finished = true;
        });

        // var task1 = new Task<PhysicsResult>(() => CalculateForces(0, quarter));
        // var task2 = new Task<PhysicsResult>(() => CalculateForces(quarter, 2 * quarter));
        // var task3 = new Task<PhysicsResult>(() => CalculateForces(2 * quarter, 3 * quarter));
        // var task4 = new Task<PhysicsResult>(() => CalculateForces(3 * quarter, normalsCount));

        // task1.RunSynchronously();
        // task2.RunSynchronously();
        // task3.RunSynchronously();
        // task4.RunSynchronously();
        
        // Task.WaitAll(task1, task2, task3, task4);
        //
        // PhysicsResult physicsResult = task1.Result;
        // PhysicsResult physicsResult2 = task2.Result;
        // PhysicsResult physicsResult3 = task3.Result;
        // PhysicsResult physicsResult4 = task4.Result;
        //
        // if (rb != null)
        // {
        //     int underwaterVertsCount = physicsResult.UnderwaterVerts
        //         + physicsResult2.UnderwaterVerts
        //         + physicsResult3.UnderwaterVerts
        //         + physicsResult4.UnderwaterVerts
        //         ;
        //
        //     // Drag for percentage underwater
        //     rb.drag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
        //     rb.angularDrag = ((underwaterVertsCount) / (float) vertsLength) * dragScalar;
        //
        //     // Apply calculated net forces to rigidbody 
        //     rb.AddForce(physicsResult.NetForce, ForceMode.Force);
        //     rb.AddTorque(physicsResult.NetTorque, ForceMode.Force);
        //
        //     rb.AddForce(physicsResult2.NetForce, ForceMode.Force);
        //     rb.AddTorque(physicsResult2.NetTorque, ForceMode.Force);
        //     
        //     rb.AddForce(physicsResult3.NetForce, ForceMode.Force);
        //     rb.AddTorque(physicsResult3.NetTorque, ForceMode.Force);
        //     
        //     rb.AddForce(physicsResult4.NetForce, ForceMode.Force);
        //     rb.AddTorque(physicsResult4.NetTorque, ForceMode.Force);
        // }
    }

    public class PhysicsResult
    {
        public Vector3 NetForce;
        public Vector3 NetTorque;
        public int UnderwaterVerts;

        public PhysicsResult()
        {
            ClearState();
        }

        public void ClearState()
        {
            NetForce = Vector3.zero;
            NetTorque = Vector3.zero;
            UnderwaterVerts = 0;
        }
    }

    private PhysicsResult CalculateForces(int startIndex, int endIndex)
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
                result.NetForce += forceAmount;

                //Torque is the cross product of the distance to center of mass and the force vector.
                result.NetTorque += Vector3.Cross(forcePosition - worldPosition, forceAmount);

                result.UnderwaterVerts++;
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