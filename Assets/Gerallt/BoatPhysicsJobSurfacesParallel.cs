using System;
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
    public struct BoatPhysicsJobSurfacesParallel : IJobParallelForTransform
    {
        public float waterLineHack;

        // Changable parameters.
        public bool isRunning;
        public float dt;
        public float myLastTime;
        public float forceScalar;
        public float areaScalar;
        public float waterDepthScalar;
        public float cullAngle;
        public bool dontCullTopNormals;
        public float mass;
        public Vector3 inertiaTensor;
        public Vector3 aerodynamicDrag;
        public float dragScalar;

        // Computed results.
        public NativeArray<Vector3> velocities;

        public NativeArray<Vector3> angularVelocities;
        //public TransformAccessArray transformAccessArray;
        //public int NumBoats;

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
            return (vw0.y < waterLine) && (vw1.y < waterLine) && (vw2.y < waterLine);
        }

        // private DateTime lastTime;
        // private DateTime currentTime;
        //
        // private float GetDeltaTime() // DateTime doesn't work with Burst Compiler, Unity's Time.time isn't thread safe
        // {
        //     currentTime = DateTime.Now;
        //     long difference = currentTime.Ticks - lastTime.Ticks;
        //     float deltaTime = difference / 10000000.0f;
        //     lastTime = currentTime;
        //     return deltaTime;
        // }

        public void Execute(int boatIndex, TransformAccess boatTransformAccess)
        {
            // lastTime = DateTime.Now;
            // currentTime = DateTime.Now;

            //while (isRunning)
            {
                //dt = GetDeltaTime(); // Custom delta time test
                //dt = 0.01f;
                
                // Calculate Forces for Buoyancy
                Quaternion transformRotation = boatTransformAccess.rotation;
                Vector3 worldPosition = boatTransformAccess.position;

                quaternion transformRotationSimd = quaternion(transformRotation.x, transformRotation.y,
                    transformRotation.z, transformRotation.w);
                float3 worldPositionSimd = float3(worldPosition.x, worldPosition.y, worldPosition.z);

                float3 netForces = float3.zero;
                float3 netTorques = float3.zero;
                int underwaterVerts = 0;
                float invMaxSurfaceArea = 1.0f / maxSurfaceArea;
                float oneThird = 1.0f / 3.0f;

                // Testing new surface area force contributions
                for (var faceIndex = 0;
                     faceIndex <= facesSimd.Length - 3;
                     faceIndex += 3) // Face is always a triangle.
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
                        float triangleArea =
                            polygonAreasSimd[
                                (int) (faceIndex * oneThird)]; // Use precalculated polygon area to save time. 
                        float areaRatio =
                            triangleArea * invMaxSurfaceArea; // float areaRatio = triangleArea / maxSurfaceArea;
                        float forceMagnitude = ((forceScalar) * (areaRatio * areaScalar)) * dt;

                        for (var vertIndex = 0; vertIndex < 3; vertIndex++)
                        {
                            var index = facesSimd[faceIndex + vertIndex];
                            float3 vertPos = vertsSimd[index];
                            float3 vertWorldPos = worldPositionSimd + mul(transformRotationSimd, vertPos);
                            float waterDepth = (1.0f + waterLineHack) - vertWorldPos.y;
                            //if(waterDepth > 1.0f + waterLineHack) waterDepth = 1.0f + waterLineHack;

                            if (vertWorldPos.y < waterLineHack)
                            {
                                Vector3 normal = normalsSimd[index];

                                if (dontCullTopNormals ||
                                    Vector3.Dot((transformRotation * normal), Vector3.down) > cullAngle)
                                {
                                    //float3 forceAmount = (transformRotation * (-normal)) * (forceMagnitude);
                                    float3 forceAmount = (transformRotation * (-normal)) *
                                                         (forceMagnitude * (waterDepth * waterDepthScalar));
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

                // Apply drag force proportional to underwater verts
                float dragForce = 0;
                if (underwaterVerts > 0)
                {
                    // Drag for percentage underwater
                    float dragCoefficient = ((underwaterVerts / (float) vertsLength) * this.dragScalar);
                    dragForce = dragCoefficient;
                }


                // Apply computed forces to transform directly using TransformAccess.
                Vector3 linearVelocity = velocities[boatIndex];
                Vector3 angularVelocity = angularVelocities[boatIndex];

                // Apply gravity force
                //Vector3 gravityForce = _transform.worldToLocalMatrix * Physics.gravity;
                Vector3 gravityForce = Physics.gravity;

                // Apply forces to boat transform.
                float invMass = (1.0f / mass);
                Vector3 acceleration = netForces * invMass; // F=ma, a=F/m
                Vector3 angularAcceleration = netTorques * invMass; // F=ma, a=F/m
                Vector3 inertiaRotated = inertiaTensor * mass; //HACK: combining mass and inertia
                Vector3 inertiaRotInv = new Vector3(1.0f / inertiaRotated.x, 1.0f / inertiaRotated.y,
                    1.0f / inertiaRotated.z);
                //angularAcceleration = Vector3.Scale(netTorques,  inertiaRotInv); // F=ma, a=F/m, Apply inertia and mass

                // Apply acceleration due to gravity
                acceleration += gravityForce;

                // Integrate to find new velocity
                Vector3 velocityChange = acceleration * dt; // linear velocity
                Vector3 angularVelocityChange = angularAcceleration * dt; // rotational velocity

                linearVelocity += velocityChange;
                angularVelocity += angularVelocityChange;


                // Vector3 alignmentTorque = Vector3.Cross(linearVelocity.normalized,  boatTransformAccess.forward.normalized); // Align to new direction
                // Vector3 alignmentVelocityChange = (alignmentTorque * invMass) * dt;
                // angularVelocity += alignmentVelocityChange;

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
                boatTransformAccess.position += positionDelta;

                // Integrate to find new rotation
                // Quaternion rotationDelta = Quaternion.Euler(angularVelocity * dt); // This is simple but it doesn't work
                // //Quaternion rotationDelta = Quaternion.LookRotation(angularVelocity * dt);
                // rotationDelta.Normalize();
                // boatTransformAccess.rotation *= rotationDelta;

                Quaternion r = boatTransformAccess.rotation;
                Quaternion w = new Quaternion(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0);
                Quaternion newRotation = Buoyancy.Add(Buoyancy.Scale(w, dt * 0.5f) * r, r);
                newRotation.Normalize();
                boatTransformAccess.rotation = newRotation;

                if (angularVelocity != Vector3.zero)
                {
                    // lerp quaternion from current towards angularVelocity to make it point to new forward
                    boatTransformAccess.rotation = Quaternion.Lerp(boatTransformAccess.rotation, Quaternion.LookRotation(angularVelocity), dt);
                }

                // Update Forward Vector based on new velocity (Align to new direction)
                //_transform.forward = _transform.position + Vector3.Slerp(_transform.forward.normalized, linearVelocity.normalized, dt);
                //_transform.forward += Vector3.Slerp(_transform.forward.normalized, linearVelocity.normalized, dt);

                // Apply drag dampening force
                // linearVelocity += velocityChange;
                // linearVelocity *= velocityDampening;
                // angularVelocity += angularVelocityChange;
                // angularVelocity *= velocityDampening;

                velocities[boatIndex] = linearVelocity;
                angularVelocities[boatIndex] = angularVelocity;
            }
        }
    }
}