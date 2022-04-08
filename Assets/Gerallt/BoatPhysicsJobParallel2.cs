using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using static Unity.Mathematics.math;
using Unity.Mathematics;
using float3 = Unity.Mathematics.float3;
using float4 = Unity.Mathematics.float4;
using UnityEngine;
using UnityEngine.Jobs;
using quaternion = Unity.Mathematics.quaternion;

namespace Gerallt
{
    [BurstCompile]
    public struct BoatPhysicsJobParallel2 : IJobParallelFor
    {
        /// <summary>
        /// Te number of jobs to execute in parallel.
        /// Changing this value automatically redistributes work size.
        /// </summary>
        public const int NumJobs = 1;
        public const int NumBoats = 500;
        
        public float waterLineHack;
        
        // Changable parameters.
        public float boatMass;
        public float forceScalar;
        public float dragScalar;
        
        // Computed results.
        public NativeArray<float3> positions;
        public NativeArray<quaternion> rotations;
        public NativeArray<bool> isDirty;
        
        [ReadOnly]
        public NativeArray<float3> vertsSimd;
        
        [ReadOnly]
        public NativeArray<float3> normalsSimd;

        [ReadOnly]
        public int normalsLength;
        
        [ReadOnly]
        public int vertsLength;
        
        public void Execute(int jobId)
        {
            int boatsPerJob = (NumBoats / NumJobs);
            float boatMassInverse = 1.0f / boatMass;
            
            //while (true)
            {
                // TODO: Calculate correct "fixed" delta time
                int waitTimeMS = 1;
                float waitTimeSeconds = waitTimeMS / 1000.0f;
                float dt = waitTimeSeconds; // HACK: Need to also accumulate lost time (time it takes physics bellow to run)
                
                //Thread.Sleep(waitTimeMS); // Can't run Thread.Sleep inside a job
                
                for (int boatId = 0; boatId < boatsPerJob; boatId++)
                {
                    int boatIndex = boatId * jobId;

                    if (boatIndex >= NumBoats)
                        continue;
                    
                    isDirty[0] = false;
                    
                    // // Calculate Forces for Buoyancy
                    // float3 transformPositionSimd = positions[boatIndex];
                    // quaternion transformRotationSimd = rotations[boatIndex];
                    // float worldY = transformPositionSimd.y;
                    //
                    // float3 netForces = float3.zero;
                    // float3 netTorques = float3.zero;
                    // int underwaterVerts = 0;
                    //
                    // for (int index = 0; index < normalsLength; index++)
                    // {
                    //     float y = worldY + mul(transformRotationSimd, vertsSimd[index]).y;
                    //     if (y < waterLineHack)
                    //     {
                    //         float3 forceAmountSimd = -normalsSimd[index] * forceScalar * dt;
                    //         float3 forcePositionSimd = vertsSimd[index];
                    //
                    //         netForces += forceAmountSimd;
                    //         netTorques += cross(forcePositionSimd, forceAmountSimd);
                    //
                    //         underwaterVerts++;
                    //     }
                    // }
                    //
                    // if (underwaterVerts > 0)
                    // {
                    //     // Apply forces to boat transform.
                    //     float3 acceleration = netForces * boatMassInverse; // F=ma, a=F/m
                    //     float3 angularAcceleration = netTorques * boatMassInverse; // F=ma, a=F/m
                    //     
                    //     // Drag for percentage underwater
                    //     float dragCoefficient = ((underwaterVerts / (float) vertsLength) * dragScalar);
                    //
                    //     float3 velocityChange = acceleration * dt; // linear velocity
                    //     float3 angularVelocity = angularAcceleration * dt; // rotational velocity
                    //
                    //     transformPositionSimd += velocityChange * dt;
                    //     transformRotationSimd = mul(transformRotationSimd, quaternion.Euler(angularVelocity * dt));
                    //     
                    //     positions[boatIndex] = transformPositionSimd;
                    //     rotations[boatIndex] = transformRotationSimd;
                    //     
                    //     isDirty[boatIndex] = true; // Flag that there have been changes
                    // }
                }
            }
        }
    }
}