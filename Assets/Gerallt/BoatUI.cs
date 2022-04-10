using UnityEngine;

public class BoatUI : MonoBehaviour
{
    public Spawner spawner;
    public Spawner spawnerRotated;
    public Spawner spawnerBigBoats;
    
    public void SpawnBoats_OnClicked()
    {
        spawner.Cleanup();
        spawnerRotated.Cleanup();
        spawnerBigBoats.Cleanup();
        
        spawner.gameObject.SetActive(true);
        spawnerRotated.gameObject.SetActive(false);
        spawnerBigBoats.gameObject.SetActive(false);
        
        spawner.enabled = true;
        spawnerRotated.enabled = false;
        spawnerBigBoats.enabled = false;
        
        spawner.SpawnAll();
    }
    
    public void SpawnBoatsRotated_OnClicked()
    {
        spawner.Cleanup();
        spawnerRotated.Cleanup();
        spawnerBigBoats.Cleanup();
        
        spawner.gameObject.SetActive(false);
        spawnerRotated.gameObject.SetActive(true);
        spawnerBigBoats.gameObject.SetActive(false);
        
        spawner.enabled = false;
        spawnerRotated.enabled = true;
        spawnerBigBoats.enabled = false;
        
        spawnerRotated.SpawnAll();
    }
    
    public void SpawnBigBoats_OnClicked()
    {
        spawner.Cleanup();
        spawnerRotated.Cleanup();
        spawnerBigBoats.Cleanup();
        
        spawner.gameObject.SetActive(false);
        spawnerRotated.gameObject.SetActive(false);
        spawnerBigBoats.gameObject.SetActive(true);
        
        spawner.enabled = false;
        spawnerRotated.enabled = false;
        spawnerBigBoats.enabled = true;
        
        spawnerBigBoats.SpawnAll();
    }
}
