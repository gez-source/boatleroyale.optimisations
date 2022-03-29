using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SplashManager : MonoBehaviour
{
	public ParticleSystem SplashesParticleSystem;
	public AudioManager AudioManager;
	ParticleSystem.EmitParams emitParams;
	public AudioClip splashClip1;
	public AudioClip lightSplashClip;

	public List<ParticleCollisionEvent> collisionEvents;


	// Use this for initialization
	void Start()
	{
		Buoyancy.OnSplash += OnSplash;
		Buoyancy.OnDestroyed += SplashSourceOnDestroyed;

		emitParams = new ParticleSystem.EmitParams();

		collisionEvents = new List<ParticleCollisionEvent>();

	}

	private void SplashSourceOnDestroyed(GameObject obj)
	{
		//        Buoyancy.OnSplash -=;
	}

	private void OnSplash(GameObject o, Vector3 worldPos, Vector3 velocity)
	{
		if (Random.value > 0.55f)
		{
			emitParams.position = worldPos + new Vector3(0, 0.1f, 0);
			emitParams.velocity = new Vector3(velocity.x, -velocity.y, velocity.z) / 2f;
			SplashesParticleSystem.Emit(emitParams, 1);

			float velocitySqrMagnitude = velocity.sqrMagnitude;
			if(velocitySqrMagnitude > 10f)
			{
				AudioManager.Play(worldPos, splashClip1, velocitySqrMagnitude / 15f, velocitySqrMagnitude / 235f, 0.2f, 5);
			}
		}
	}

	void OnParticleCollision(GameObject other)
	{
		// Unity example stuff
		int numCollisionEvents = SplashesParticleSystem.GetCollisionEvents(other, collisionEvents);
		//		print(numCollisionEvents);
		int i = 0;

		while (i < numCollisionEvents)
		//		while (i < 1)
		{
			float velocity = collisionEvents[i].velocity.sqrMagnitude;
			if (velocity > 20f)
			{
//				AudioManager.Play(collisionEvents[i].intersection, lightSplashClip, 1f, 0.3f + velocity / 135f, 0.2f, 2);
				AudioManager.Play(collisionEvents[i].intersection, lightSplashClip, 0.3f + velocity / 135f, 0.7f, 0.2f, 2);
			}
			i++;
		}
	}
}