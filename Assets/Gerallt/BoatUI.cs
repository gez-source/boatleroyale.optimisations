using UnityEngine;

public class BoatUI : MonoBehaviour
{
    public Spawner spawner;
    public Spawner spawnerRotated;
    
    public void SpawnBoats_OnClicked()
    {
        spawner.Cleanup();
        spawnerRotated.Cleanup();
        
        spawner.gameObject.SetActive(true);
        spawnerRotated.gameObject.SetActive(false);

        spawner.enabled = true;
        spawnerRotated.enabled = false;
        
        spawner.SpawnAll();
    }
    
    public void SpawnBoatsRotated_OnClicked()
    {
        spawner.Cleanup();
        spawnerRotated.Cleanup();
        
        spawner.gameObject.SetActive(false);
        spawnerRotated.gameObject.SetActive(true);
        
        spawner.enabled = false;
        spawnerRotated.enabled = true;
        
        spawnerRotated.SpawnAll();
    }
}
