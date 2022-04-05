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
    public struct BoatPhysicsJob : IJob
    {
        public float waterLineHack;
        
        // Changable parameters.
        public float dt;
        public Quaternion transformRotation;
        public float worldY;
        public float forceScalar;
        
        // Computed results.
        public NativeArray<int> underwaterVertsIns;
        public NativeArray<float3> computedForces;
        
        [ReadOnly]
        public NativeArray<float3> vertsSimd;
        
        [ReadOnly]
        public NativeArray<float3> normalsSimd;

        [ReadOnly]
        public int normalsLength;
        
        public void Execute()
        {
            // Calculate Forces for Buoyancy
            quaternion transformRotationSimd = quaternion(transformRotation.x, transformRotation.y, transformRotation.z, transformRotation.w);

            float3 netForces = float3.zero;
            float3 netTorques = float3.zero;
            int underwaterVerts = 0;
            
            for (var index = 0; index < normalsLength; index++)
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

            underwaterVertsIns[0] = underwaterVerts;
            computedForces[0] = netForces;
            computedForces[1] = netTorques;
        }
    }
}