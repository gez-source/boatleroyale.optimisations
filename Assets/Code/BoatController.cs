using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoatController : MonoBehaviour
{
	public float speed;
	public Transform motor;
	public float maxMotorAngle;
	public Buoyancy Buoyancy;
	public Rigidbody Rigidbody;
	
	// Update is called once per frame
	void Update()
	{
		// Turning
		motor.rotation = transform.rotation * Quaternion.Euler(0, -Input.GetAxis("Horizontal")*maxMotorAngle, 0);
		
		// HACK: Forward motor
		if (motor.position.y < Buoyancy.waterLineHack)
		{
			//			Rigidbody.AddRelativeForce(0,0,speed * (underwaterVerts / (float)totalVerts));
			Rigidbody.AddForceAtPosition(motor.transform.forward * speed * Time.deltaTime * Input.GetAxis("Vertical"),
				motor.position + new Vector3(0, 0.25f, 0)); // HACK: Offset hack to stop the boat flipping all the time!
		}
	}
}
