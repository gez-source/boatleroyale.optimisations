using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using static Unity.Mathematics.math;
using Unity.Mathematics;
using float3 = Unity.Mathematics.float3;
using float4 = Unity.Mathematics.float4;
using UnityEngine;
using quaternion = Unity.Mathematics.quaternion;

namespace Gerallt
{
    [BurstCompile]
    public struct BoatPhysicsJobParallel : IJobParallelFor
    {
        /// <summary>
        /// Te number of jobs to execute in parallel.
        /// Changing this value automatically redistributes work size.
        /// </summary>
        public const int NUM_JOBS = 8;
        
        public float waterLineHack;
        
        // Changable parameters.
        public float dt;
        public Quaternion transformRotation;
        public float worldY;
        public float forceScalar;
        
        // Computed results.
        public NativeArray<int> underwaterVertsIns;
        public NativeArray<float3> forces;
        public NativeArray<float3> torques;
        
        [ReadOnly]
        public NativeArray<float3> vertsSimd;
        
        [ReadOnly]
        public NativeArray<float3> normalsSimd;

        [ReadOnly]
        public int normalsLength;
        
        public void Execute(int i)
        {
            // Calculate Forces for Buoyancy
            quaternion transformRotationSimd = quaternion(transformRotation.x, transformRotation.y, transformRotation.z, transformRotation.w);

            float3 netForces = float3.zero;
            float3 netTorques = float3.zero;
            int underwaterVerts = 0;

            //  Determine workload given index
            int startIndex;
            int endIndex;
            int jobSize = normalsLength / NUM_JOBS;

            if (i == 0)
            {
                startIndex = 0;
                endIndex = jobSize;
            }
            else
            {
                startIndex = jobSize * i;
                endIndex = jobSize * (i + 1);
            }

            for (int index = startIndex; index < endIndex; index++)
            {
                float y = worldY + mul(transformRotationSimd, vertsSimd[index]).y;
                //float y = worldY + (transformRotation * vertsSimd[index]).y;
                if (y < waterLineHack)
                {
                    float3 forceAmountSimd = -normalsSimd[index] * forceScalar * dt;
                    float3 forcePositionSimd = vertsSimd[index];

                    netForces += forceAmountSimd;
                    netTorques += cross(forcePositionSimd, forceAmountSimd);
                    
                    underwaterVerts++;
                }
            }

            underwaterVertsIns[i] = underwaterVerts;
            forces[i] = netForces;
            torques[i] = netTorques;
        }
    }
}