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
    public struct BoatPhysicsJobSurfaces : IJob
    {
        public float waterLineHack;

        // Changable parameters.
        public float dt;
        public Quaternion transformRotation;
        public Vector3 worldPosition;
        public float forceScalar;
        public float areaScalar;
        public float waterDepthScalar;
        public float cullAngle;
        public bool dontCullTopNormals;

        // Computed results.
        public NativeArray<int> underwaterVertsIns;
        public NativeArray<float3> computedForces;

        [ReadOnly] public NativeArray<float3> vertsSimd;

        [ReadOnly] public NativeArray<float3> normalsSimd;

        [ReadOnly] public NativeArray<float> polygonAreasSimd;

        [ReadOnly] public NativeArray<int> facesSimd;

        [ReadOnly] public int normalsLength;

        [ReadOnly] public int vertsLength;

        [ReadOnly] public float maxSurfaceArea;

        bool TriangleIntersectsWaterline(Vector3 vw0, Vector3 vw1, Vector3 vw2, Vector3 n0, Vector3 n1, Vector3 n2,
            float waterLine)
        {
            // float rayDistance = 0.01f;
            //
            // bool result = Physics.Raycast(vw0, n0, rayDistance)
            //               || Physics.Raycast(vw1, n1, rayDistance)
            //               || Physics.Raycast(vw2, n2, rayDistance);
            // return result;
            return (vw0.y < waterLine) && (vw1.y < waterLine) && (vw2.y < waterLine);
        }

        public void Execute()
        {
            // Calculate Forces for Buoyancy
            quaternion transformRotationSimd = quaternion(transformRotation.x, transformRotation.y, transformRotation.z, transformRotation.w);
            float3 worldPositionSimd = float3(worldPosition.x, worldPosition.y, worldPosition.z);

            float3 netForces = float3.zero;
            float3 netTorques = float3.zero;
            int underwaterVerts = 0;
            float invMaxSurfaceArea = 1.0f / maxSurfaceArea;
            float oneThird = 1.0f / 3.0f;

            // Testing new surface area force contributions
            for (var faceIndex = 0; faceIndex <= facesSimd.Length - 3; faceIndex += 3) // Face is always a triangle.
            {
                var index0 = facesSimd[faceIndex];
                var index1 = facesSimd[faceIndex + 1];
                var index2 = facesSimd[faceIndex + 2];

                if (index0 >= vertsLength)
                    continue;

                float3 v1 = vertsSimd[index0];
                float3 v2 = vertsSimd[index1];
                float3 v3 = vertsSimd[index2];

                float3 vw0 = worldPositionSimd + mul(transformRotationSimd, v1);
                float3 vw1 = worldPositionSimd + mul(transformRotationSimd, v2);
                float3 vw2 = worldPositionSimd + mul(transformRotationSimd, v3);

                float3 n0 = normalsSimd[index0];
                float3 n1 = normalsSimd[index1];
                float3 n2 = normalsSimd[index2];

                if (TriangleIntersectsWaterline(vw0, vw1, vw2, n0, n1, n2, waterLineHack))
                {
                    // Take into account the contribution of the surface area and each vertex
                    float triangleArea = polygonAreasSimd[(int) (faceIndex * oneThird)]; // Use precalculated polygon area to save time. 
                    float areaRatio = triangleArea * invMaxSurfaceArea; // float areaRatio = triangleArea / maxSurfaceArea;
                    float forceMagnitude = ((forceScalar) * (areaRatio * areaScalar)) * dt;

                    for (var vertIndex = 0; vertIndex < 3; vertIndex++)
                    {
                        var index = facesSimd[faceIndex + vertIndex];
                        float3 vertPos = vertsSimd[index];
                        float3 vertWorldPos = worldPositionSimd + mul(transformRotationSimd, vertPos);
                        float waterDepth = (1.0f + waterLineHack) - vertWorldPos.y;

                        if (vertWorldPos.y < waterLineHack)
                        {
                            Vector3 normal = normalsSimd[index];

                            if (dontCullTopNormals || Vector3.Dot((transformRotation * normal), Vector3.down) > cullAngle)
                            {
                                float3 forceAmount = (transformRotation * (-normal)) * (forceMagnitude * (waterDepth * waterDepthScalar));
                                float3 forcePosition = vertWorldPos;

                                netForces += forceAmount;
                                netTorques += cross(forcePosition - worldPositionSimd, forceAmount);
                                // Torque is the cross product of the distance to center of mass and the force vector.

                                underwaterVerts++;
                            }
                        }
                    }
                }
            }

            underwaterVertsIns[0] = underwaterVerts;
            computedForces[0] = netForces;
            computedForces[1] = netTorques;
        }
    }
}