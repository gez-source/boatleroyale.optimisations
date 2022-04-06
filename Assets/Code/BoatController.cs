using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class BoatController : MonoBehaviour
{
	public float speed;
	public Transform motor;
	public float maxMotorAngle;
	public Buoyancy Buoyancy;
	public Rigidbody Rigidbody;
	private Transform _transform;
	private Quaternion motorDirection = Quaternion.identity;
	private Vector3 eulerAngles = Vector3.zero;
	public Vector3 bounceBackOffset = new Vector3(0, 0.25f, 0); // HACK: Offset hack to stop the boat flipping all the time!
	
	public void Awake()
	{
		_transform = GetComponent<Transform>();
	}

	// Update is called once per frame
	void Update()
	{
		// Turning
		//motor.rotation = _transform.rotation * Quaternion.Euler(0, -Input.GetAxis("Horizontal")*maxMotorAngle, 0); // Much slower.
		
		eulerAngles.y = -Input.GetAxis("Horizontal") * maxMotorAngle;
		motorDirection.eulerAngles = eulerAngles;
		motor.rotation = _transform.rotation * motorDirection;
		
		Vector3 motorPosition = motor.position;
		
		// HACK: Forward motor
		if (motorPosition.y < Buoyancy.waterLineHack)
		{
			//			Rigidbody.AddRelativeForce(0,0,speed * (underwaterVerts / (float)totalVerts));
			
			Vector3 force = motor.forward * speed * Time.deltaTime * Input.GetAxis("Vertical");
			Vector3 forcePosition = motorPosition + bounceBackOffset;
			
			Rigidbody.AddForceAtPosition(force, forcePosition); 

			// No discernible difference in performance manually doing things:
			// Vector3 torque = Vector3.Cross(forcePosition - motorPosition, force);
			// Rigidbody.AddForce(force, ForceMode.Force);
			// Rigidbody.AddTorque(torque, ForceMode.Force);
		}
	}
}
