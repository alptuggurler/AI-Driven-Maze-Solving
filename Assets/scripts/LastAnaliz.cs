using UnityEngine;
using System.Collections;

public class LastAnaliz : MonoBehaviour
{
    public FindPathAStar_sc aStarScript;
    public QLearningPathfindingV5 qLearningScript;
    private bool isAnalyzing = false;

    void Start()
    {
        // Referansları kontrol et
        if (aStarScript == null || qLearningScript == null)
        {
            Debug.LogError("Lütfen A* ve Q-Learning script referanslarını Inspector'da atayın!");
            enabled = false;
            return;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q) && !isAnalyzing)
        {
            // Her iki algoritma için aynı rastgele noktaları belirle
            qLearningScript.RandomizeStartAndGoalV5();
            SyncStartAndGoalPoints();
        }

        if (Input.GetKeyDown(KeyCode.W) && !isAnalyzing)
        {
            // Q-Learning otomatik eğitimi başlat
            StartCoroutine(RunQLearning());
        }

        if (Input.GetKeyDown(KeyCode.E) && !isAnalyzing)
        {
            // A* algoritmasını çalıştır
            StartCoroutine(RunAStar());
        }
    }

    private void SyncStartAndGoalPoints()
    {
        // Q-Learning'den başlangıç ve bitiş noktalarını al
        MapLocation qStart = qLearningScript.GetStartPoint();
        MapLocation qGoal = qLearningScript.GetGoalPoint();

        // A* için aynı noktaları ayarla
        Vector3 startPos = new Vector3(qStart.x * aStarScript.maze.scale, 0, qStart.z * aStarScript.maze.scale);
        Vector3 goalPos = new Vector3(qGoal.x * aStarScript.maze.scale, 0, qGoal.z * aStarScript.maze.scale);

        // A* başlangıç ve bitiş noktalarını ayarla
        if (aStarScript.startNode != null) Destroy(aStarScript.startNode.marker);
        if (aStarScript.goalNode != null) Destroy(aStarScript.goalNode.marker);

        aStarScript.startNode = new PathMarker(qStart, 0, 0, 0,
            Instantiate(aStarScript.start, startPos, Quaternion.identity), null);
        aStarScript.goalNode = new PathMarker(qGoal, 0, 0, 0,
            Instantiate(aStarScript.end, goalPos, Quaternion.identity), null);
    }

    private IEnumerator RunQLearning()
    {
        isAnalyzing = true;
        Debug.Log("<color=yellow>Q-Learning analizi başlatılıyor...</color>");
        
        // Q-Learning otomatik eğitimi başlat
        qLearningScript.StartAutoTraining();
        
        // Q-Learning tamamlanana kadar bekle
        while (qLearningScript.IsTraining())
        {
            yield return null;
        }
        
        isAnalyzing = false;
    }

    private IEnumerator RunAStar()
    {
        isAnalyzing = true;
        Debug.Log("<color=yellow>A* analizi başlatılıyor...</color>");
        
        // A* algoritmasını çalıştır
        aStarScript.RunCompleteAStar();
        
        yield return new WaitForSeconds(0.1f);
        isAnalyzing = false;
    }
} 