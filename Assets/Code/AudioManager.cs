using UnityEngine;

// Cams quick and dirty Audio pooling manager
public class AudioManager : MonoBehaviour
{
	public AudioSource[] AudioSources;
	public int maxAudioSources;
	private float lastTrigTime;


	// Use this for initialization
	void Start()
	{
		lastTrigTime = Time.timeSinceLevelLoad;

		AudioSources = new AudioSource[maxAudioSources];
		for (int i = 0; i < maxAudioSources; i++)
		{
			GameObject source = new GameObject("AudioSource");
			source.transform.parent = transform;
			AudioSources[i] = source.AddComponent<AudioSource>();
			AudioSources[i].spatialBlend = 1f;
			AudioSources[i].playOnAwake = false;
		}
	}

	public void Play(AudioClip clip)
	{
		Play(Vector3.zero, clip, 1f, 1f, 0, 10);
	}

	public void Play(Vector3 position, AudioClip clip, float volume, float pitch, float minRetrigTime, int priority)
	{
		// Stop quick successive calls to play
		if (Time.timeSinceLevelLoad < lastTrigTime + minRetrigTime +UnityEngine.Random.Range(0,0.1f))
			return;
		lastTrigTime = Time.timeSinceLevelLoad;

		// Find a free AudioSource
		for(var index = 0; index < AudioSources.Length; index++)
		{
			AudioSource audioSource = AudioSources[index];

			// Override quieter sound
			if(!audioSource.isPlaying)// || volume > audioSource.volume || priority > audioSource.priority)
			{
				audioSource.priority = priority;
				audioSource.transform.position = position;
				audioSource.clip = clip;
				audioSource.volume = volume;
				audioSource.pitch = 0.75f + pitch + UnityEngine.Random.Range(-0.1f, 0.1f);
				audioSource.Play();
				break;
			}
		}
	}
}
