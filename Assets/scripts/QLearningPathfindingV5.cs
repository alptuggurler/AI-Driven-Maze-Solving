using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class QLearningPathfindingV5 : MonoBehaviour
{
    public Maze maze; // Maze referansı
    public Material pathMaterial; // Yolu işaretlemek için material
    public GameObject startMarker; // Başlangıç işareti
    public GameObject goalMarker; // Hedef işareti
    public GameObject pathMarker; // Yol işareti
    public GameObject deadEndMarker; // Yeni: Çıkmaz yol işaretçisi

    private MapLocation startPoint;
    private MapLocation goalPoint;
    private bool isTraining = false; // Eğitim durumu
    private int iterationCount = 0; // Iterasyon sayacı
    [SerializeField] private float initialLearningRate = 0.8f;
    [SerializeField] private float minLearningRate = 0.1f;
    private int totalTrainingCount = 0; // Toplam eğitim sayısı
    [SerializeField] private float discountFactor = 0.9f; // İndirim faktörü
    [SerializeField] private float explorationRate = 0.2f; // Keşif oranı
    [SerializeField] private int maxTrainingIterations = 10000;
    [SerializeField] private float minExplorationRate = 0.01f;
    [SerializeField] private int maxTrainingAttempts = 1000; // Maksimum eğitim denemesi
    private bool isAutoTraining = false;

    private Dictionary<MapLocation, float[]> qTable = new Dictionary<MapLocation, float[]>(); // Q-Matrisi
    private HashSet<MapLocation> visitedStates = new HashSet<MapLocation>(); // Ziyaret edilen durumlar
    private List<GameObject> pathMarkers = new List<GameObject>(); // Yol işaretçileri
    private List<GameObject> persistentMarkers = new List<GameObject>(); // Kalıcı işaretçiler (başlangıç ve bitiş)
    private List<MapLocation> bestPathFound = new List<MapLocation>();
    private int bestStepCount = int.MaxValue;
    private HashSet<MapLocation> temporaryWalls = new HashSet<MapLocation>();
    private List<GameObject> deadEndMarkers = new List<GameObject>(); // Yeni: Çıkmaz yol işaretçileri listesi

    private float globalStartTime;

    void Start()
    {
        // Bileşenlerin varlığını kontrol et
        if (maze == null || pathMarker == null || startMarker == null || goalMarker == null)
        {
            Debug.LogError("Gerekli bileşenler atanmamış! Lütfen Unity Inspector'da tüm referansları kontrol edin.");
            enabled = false;
            return;
        }
        
        // Başlangıçta önceki Q-Table'ı yükle
        LoadQTable();
    }

    // Başlangıç ve bitiş noktalarını rastgele seçme
    public void RandomizeStartAndGoalV5()
    {
        ClearAllMarkers(); // Tüm işaretçileri temizle
        temporaryWalls.Clear(); // Geçici duvarları temizle
        do
        {
            startPoint = new MapLocation(Random.Range(1, maze.width - 1), Random.Range(1, maze.depth - 1));
        } while (maze.map[startPoint.x, startPoint.z] != 0);

        do
        {
            goalPoint = new MapLocation(Random.Range(1, maze.width - 1), Random.Range(1, maze.depth - 1));
        } while (maze.map[goalPoint.x, goalPoint.z] != 0 || goalPoint.Equals(startPoint));

        Debug.Log($"Başlangıç: {startPoint.x}, {startPoint.z} | Bitiş: {goalPoint.x}, {goalPoint.z}");
        VisualizeStartAndGoalV5();
    }

    // Başlangıç ve bitiş noktalarını görselleştirme
    void VisualizeStartAndGoalV5()
    {
        foreach (var marker in persistentMarkers)
        {
            Destroy(marker);
        }
        persistentMarkers.Clear();

        Vector3 startPos = new Vector3(startPoint.x * maze.scale, 0.5f, startPoint.z * maze.scale);
        GameObject start = Instantiate(startMarker, startPos, Quaternion.identity);
        persistentMarkers.Add(start);

        Vector3 goalPos = new Vector3(goalPoint.x * maze.scale, 0.5f, goalPoint.z * maze.scale);
        GameObject goal = Instantiate(goalMarker, goalPos, Quaternion.identity);
        persistentMarkers.Add(goal);

        Debug.Log($"Başlangıç noktası: ({startPoint.x}, {startPoint.z})");
        Debug.Log($"Bitiş noktası: ({goalPoint.x}, {goalPoint.z})");
        
        // Hedefin etrafındaki geçerli yolları göster
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dz == 0) continue; // Hedefin kendisini atla
                if (Mathf.Abs(dx) + Mathf.Abs(dz) == 2) continue; // Çapraz noktaları atla
                
                int x = goalPoint.x + dx;
                int z = goalPoint.z + dz;
                
                if (x >= 0 && x < maze.width && z >= 0 && z < maze.depth)
                {
                    if (maze.map[x, z] == 0)
                    {
                        //Debug.Log($"Hedefe komşu geçerli yol: ({x}, {z})");
                    }
                }
            }
        }
    }

    private bool IsGoalAdjacent(MapLocation state, out int directionToGoal)
    {
        // Hedefin komşu olup olmadığını kontrol et ve yönünü döndür
        directionToGoal = -1;
        for (int i = 0; i < maze.directions.Count; i++)
        {
            MapLocation checkPos = new MapLocation(
                state.x + maze.directions[i].x,
                state.z + maze.directions[i].z
            );
            
            if (checkPos.Equals(goalPoint))
            {
                directionToGoal = i;
                return true;
            }
        }
        return false;
    }

    private bool IsNextToGoal(MapLocation state)
    {
        // Sadece Manhattan mesafesine bak
        int distanceToGoal = Mathf.Abs(state.x - goalPoint.x) + Mathf.Abs(state.z - goalPoint.z);
        //Debug.Log($"Mevcut Konum: ({state.x}, {state.z}), Hedef: ({goalPoint.x}, {goalPoint.z}), Mesafe: {distanceToGoal}");
        
        // Mesafe 1 ise başarılı say
        return distanceToGoal == 1;
    }

    // Q-Learning eğitimi
    void TrainAgentV5()
    {
        // Manuel eğitimde geçici duvarları temizle
        temporaryWalls.Clear();
        TrainAgentV5WithoutClearingWalls();
    }

    // Geçici duvarları temizlemeden eğitim yapan versiyon
    void TrainAgentV5WithoutClearingWalls()
    {
        isTraining = true;
        iterationCount = 0;  // Her eğitimde sıfırla
        totalTrainingCount++;
        
        int maxAttempts = 1;
        bool foundPath = false;
        
        for(int attempt = 0; attempt < maxAttempts && !foundPath; attempt++)
        {
            MapLocation currentState = startPoint;
            List<MapLocation> currentPath = new List<MapLocation> { currentState };
            visitedStates.Clear();
            visitedStates.Add(currentState);
            int stepsToGoal = 0;
            
            while (true)
            {
                iterationCount++;  // Her adımda iterasyon sayısını artır
                
                // Mevcut konumdan gidebileceği yönleri kontrol et
                List<MapLocation> availableDirections = new List<MapLocation>();
                foreach (var direction in maze.directions)
                {
                    MapLocation nextPos = new MapLocation(
                        currentState.x + direction.x,
                        currentState.z + direction.z
                    );
                    
                    if (nextPos.x >= 0 && nextPos.x < maze.width && 
                        nextPos.z >= 0 && nextPos.z < maze.depth && 
                        maze.map[nextPos.x, nextPos.z] == 0 &&
                        !visitedStates.Contains(nextPos) &&
                        !temporaryWalls.Contains(nextPos))
                    {
                        availableDirections.Add(nextPos);
                    }
                }

                // Eğer gidecek yer kalmadıysa, bu bir çıkmaz yol
                if (availableDirections.Count == 0)
                {
                    temporaryWalls.Add(currentState);
                    
                    // Çıkmaz yol için geriye dönük ceza uygula
                    float deadEndPenalty = -100.0f;
                    MapLocation previousState = currentPath[currentPath.Count - 2]; // Son durumdan önceki durum
                    
                    // Son 3 durumu geriye dönük cezalandır
                    for(int i = currentPath.Count - 1; i >= Mathf.Max(0, currentPath.Count - 3); i--)
                    {
                        MapLocation state = currentPath[i];
                        if (!qTable.ContainsKey(state))
                        {
                            qTable[state] = new float[4];
                        }
                        
                        float[] stateQValues = qTable[state];
                        // Tüm aksiyonları cezalandır, ceza geriye gittikçe azalsın
                        float statePenalty = deadEndPenalty / (currentPath.Count - i);
                        for (int j = 0; j < stateQValues.Length; j++)
                        {
                            stateQValues[j] += statePenalty;
                        }
                    }

                   // Debug.Log($"Çıkmaz yol tespit edildi: ({currentState.x}, {currentState.z}), İterasyon: {iterationCount}"); //Bu Debug Kapat
                    
                    // Çıkmaz yolu görselleştir
                    //Vector3 position = new Vector3(currentState.x * maze.scale, 0.5f, currentState.z * maze.scale);
                    //GameObject marker = Instantiate(deadEndMarker, position, Quaternion.identity);
                    //deadEndMarkers.Add(marker);
                    break;
                }

                
                bool isNextToGoal = IsNextToGoal(currentState);
                if (isNextToGoal)
                {
                    currentPath.Add(goalPoint);
                    stepsToGoal++;
                    
                    Debug.Log($"Çözüm bulundu! Toplam adım sayısı: {stepsToGoal}, İterasyon: {iterationCount}");
                    float currentDuration = Time.time - globalStartTime;
                    Debug.Log(qTable);
                    Debug.Log($"<color=#FFD700><b>Geçen Süre: {currentDuration} saniye Sonuc2 V5</b></color>");
                    // Yolu görselleştir
                    ClearMarkersV5();
                    foreach (var location in currentPath)
                    {
                        Vector3 position = new Vector3(location.x * maze.scale, 0.5f, location.z * maze.scale);
                        GameObject marker = Instantiate(pathMarker, position, Quaternion.identity);
                        pathMarkers.Add(marker);
                    }
                    foundPath = true;
                    
                    // Otomatik eğitimi durdur
                    if (isAutoTraining)
                    {
                        StopAllCoroutines();
                        isAutoTraining = false;
                        Debug.Log($"Çözüm bulundu! Otomatik eğitim durduruldu. (İterasyon: {iterationCount})");
                    }
                    break;
                }

                int action = ChooseAction(currentState);
                MapLocation nextState = GetNextStateV5(currentState, action);
                
                // Geçici duvarları kontrol et
                if (temporaryWalls.Contains(nextState))
                {
                    continue;
                }

                float reward = GetReward(nextState, isNextToGoal);  // IsNextToGoal sonucunu kullan

                if (!qTable.ContainsKey(currentState))
                {
                    qTable[currentState] = new float[4];
                }

                float[] currentQValues = qTable[currentState];
                float maxNextQ = 0f;

                if (qTable.ContainsKey(nextState))
                {
                    maxNextQ = qTable[nextState].Max();
                }
                else
                {
                    qTable[nextState] = new float[4];
                }

                float currentLearningRate = GetDynamicLearningRate();
                currentQValues[action] += currentLearningRate * (reward + discountFactor * maxNextQ - currentQValues[action]);

                if (visitedStates.Contains(nextState))
                {
                    break;
                }

                currentPath.Add(nextState);
                visitedStates.Add(nextState);
                currentState = nextState;
                stepsToGoal++;
            }

            if (foundPath)
            {
                Debug.Log($"En kısa yol bulundu! Toplam adım sayısı: {stepsToGoal}, İterasyon: {iterationCount} Sonuc2 v5" );
                break;
            }
        }

        SaveQTable();
        isTraining = false;
    }

    private float GetDynamicExplorationRate()
    {
        float dynamicRate = Mathf.Max(minExplorationRate, 
            explorationRate * (1f - (float)iterationCount / maxTrainingIterations));
        return dynamicRate;
    }

    int ChooseAction(MapLocation state)
    {
        // Hedefe olan Manhattan mesafesini hesapla
        int distanceToGoal = Mathf.Abs(state.x - goalPoint.x) + 
                            Mathf.Abs(state.z - goalPoint.z);

        // Hedefe yakınken (3 birim veya daha az) hedefe doğru hareket etmeyi zorla
        if (distanceToGoal <= 3)
        {
            List<int> possibleActions = new List<int>();
            float bestDistance = float.MaxValue;
            int bestAction = -1;

            for (int i = 0; i < maze.directions.Count; i++)
            {
                MapLocation nextPos = new MapLocation(
                    state.x + maze.directions[i].x,
                    state.z + maze.directions[i].z
                );
                
                // Geçerli hareket mi kontrol et (geçici duvarları da kontrol et)
                if (nextPos.x >= 0 && nextPos.x < maze.width && 
                    nextPos.z >= 0 && nextPos.z < maze.depth && 
                    maze.map[nextPos.x, nextPos.z] == 0 &&
                    !visitedStates.Contains(nextPos) &&
                    !temporaryWalls.Contains(nextPos))  // Geçici duvar kontrolü eklendi
                {
                    float newDistance = Mathf.Abs(nextPos.x - goalPoint.x) + 
                                      Mathf.Abs(nextPos.z - goalPoint.z);
                    
                    // Hedefe en yakın hareketi bul
                    if (newDistance < bestDistance)
                    {
                        bestDistance = newDistance;
                        bestAction = i;
                    }
                }
            }

            // Eğer geçerli bir hareket bulunduysa ve hedefe yaklaştırıyorsa
            if (bestAction != -1 && bestDistance < distanceToGoal)
            {
                return bestAction;
            }
        }

        // Normal hareket seçimi
        float exploreRate = GetDynamicExplorationRate();
        
        if (Random.value < exploreRate * (1f - (float)totalTrainingCount / maxTrainingAttempts))
        {
            // Keşif modu: Geçici duvar olmayan yönleri tercih et
            List<int> safeActions = new List<int>();  // Geçici duvar olmayan yönler
            List<int> fallbackActions = new List<int>();  // Diğer geçerli yönler
            
            for (int i = 0; i < maze.directions.Count; i++)
            {
                MapLocation nextPos = new MapLocation(
                    state.x + maze.directions[i].x,
                    state.z + maze.directions[i].z
                );
                
                if (nextPos.x >= 0 && nextPos.x < maze.width && 
                    nextPos.z >= 0 && nextPos.z < maze.depth && 
                    maze.map[nextPos.x, nextPos.z] == 0 &&
                    !visitedStates.Contains(nextPos))
                {
                    if (!temporaryWalls.Contains(nextPos))
                    {
                        safeActions.Add(i);  // Geçici duvar olmayan yönleri öncelikle ekle
                    }
                    else
                    {
                        fallbackActions.Add(i);  // Geçici duvar olan yönleri yedek olarak tut
                    }
                }
            }
            
            // Önce geçici duvar olmayan yönleri dene
            if (safeActions.Count > 0)
            {
                return safeActions[Random.Range(0, safeActions.Count)];
            }
            // Eğer mecbursak diğer yönleri kullan
            else if (fallbackActions.Count > 0)
            {
                return fallbackActions[Random.Range(0, fallbackActions.Count)];
            }
            return Random.Range(0, maze.directions.Count);
        }
        else
        {
            // Q-Table'dan en iyi hareketi seç
            if (qTable.ContainsKey(state))
            {
                float[] qValues = qTable[state];
                float maxQ = float.MinValue;
                List<int> bestActions = new List<int>();
                List<int> fallbackActions = new List<int>();

                for (int i = 0; i < qValues.Length; i++)
                {
                    MapLocation nextPos = new MapLocation(
                        state.x + maze.directions[i].x,
                        state.z + maze.directions[i].z
                    );
                    
                    if (nextPos.x >= 0 && nextPos.x < maze.width && 
                        nextPos.z >= 0 && nextPos.z < maze.depth && 
                        maze.map[nextPos.x, nextPos.z] == 0 &&
                        !visitedStates.Contains(nextPos))
                    {
                        if (!temporaryWalls.Contains(nextPos))
                        {
                            if (qValues[i] > maxQ)
                            {
                                maxQ = qValues[i];
                                bestActions.Clear();
                                bestActions.Add(i);
                            }
                            else if (Mathf.Approximately(qValues[i], maxQ))
                            {
                                bestActions.Add(i);
                            }
                        }
                        else
                        {
                            fallbackActions.Add(i);
                        }
                    }
                }

                if (bestActions.Count > 0)
                {
                    return bestActions[Random.Range(0, bestActions.Count)];
                }
                else if (fallbackActions.Count > 0)
                {
                    return fallbackActions[Random.Range(0, fallbackActions.Count)];
                }
            }
            return Random.Range(0, maze.directions.Count);
        }
    }

    float GetReward(MapLocation state, bool isNextToGoal)
    {
        if (state.Equals(goalPoint) || isNextToGoal)
            return 1000.0f;
        
        float distanceToGoal = Mathf.Abs(state.x - goalPoint.x) + Mathf.Abs(state.z - goalPoint.z);
        
        if (maze.map[state.x, state.z] == 1)
            return -100.0f;
        
        return -0.1f * distanceToGoal;
    }

    MapLocation GetNextStateV5(MapLocation state, int action)
    {
        MapLocation direction = maze.directions[action];
        MapLocation nextState = new MapLocation(state.x + direction.x, state.z + direction.z);

        if (nextState.Equals(goalPoint))
        {
            return goalPoint;
        }

        // Geçici duvarları da kontrol et
        if (nextState.x < 0 || nextState.x >= maze.width || 
            nextState.z < 0 || nextState.z >= maze.depth || 
            maze.map[nextState.x, nextState.z] == 1 ||
            temporaryWalls.Contains(nextState))
        {
            return state;
        }
        return nextState;
    }

    void VisualizeBestPathV5()
    {
        ClearMarkersV5();

        MapLocation currentState = startPoint;
        List<MapLocation> bestPath = new List<MapLocation> { currentState };

        int maxIterations = 1000;
        int iterationCount = 0;

        while (!currentState.Equals(goalPoint))
        {
            if (!qTable.ContainsKey(currentState))
            {
                Debug.LogWarning("Q-Tablosunda eksik durum tespit edildi. Görselleştirme tamamlanamıyor.");
                return;
            }

            int bestAction = System.Array.IndexOf(qTable[currentState], qTable[currentState].Max());
            MapLocation nextState = GetNextStateV5(currentState, bestAction);

            if (nextState.Equals(currentState))
            {
                Debug.LogWarning("Aynı durumda sıkışıldı. Döngü sonlandırılıyor.");
                return;
            }

            currentState = nextState;
            bestPath.Add(currentState);

            iterationCount++;
            if (iterationCount > maxIterations)
            {
                Debug.LogError("Döngü iterasyon sınırını aştı. Sonsuz döngüden kaçınıldı.");
                return;
            }
        }
    }

    void VisualizeLastPathV5()
    {
        ClearMarkersV5();

        foreach (var location in visitedStates)
        {
            Vector3 position = new Vector3(location.x * maze.scale, 0.5f, location.z * maze.scale);
            GameObject marker = Instantiate(pathMarker, position, Quaternion.identity);
            pathMarkers.Add(marker);
        }

        //Debug.Log("Son yol görselleştirildi.");
    }

    void ClearMarkersV5()
    {
        // Sadece yol işaretçilerini temizle
        foreach (var marker in pathMarkers)
        {
            Destroy(marker);
        }
        pathMarkers.Clear();

        // Çıkmaz yol işaretçilerini temizleme
        // foreach (var marker in deadEndMarkers)
        // {
        //     Destroy(marker);
        // }
        // deadEndMarkers.Clear();

        //Debug.Log("İşaretçiler temizlendi.");
    }

    void SaveQTable()
    {
        try
        {
            string qTableJson = JsonUtility.ToJson(new SerializableQTable(qTable));
            PlayerPrefs.SetString("QTableData", qTableJson);
            PlayerPrefs.Save();
            //Debug.Log("Q-Table başarıyla kaydedildi.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Q-Table kaydedilirken hata oluştu: {e.Message}");
        }
    }

    void LoadQTable()
    {
        try
        {
            if (PlayerPrefs.HasKey("QTableData"))
            {
                string qTableJson = PlayerPrefs.GetString("QTableData");
                SerializableQTable loadedData = JsonUtility.FromJson<SerializableQTable>(qTableJson);
                if (loadedData != null)
                {
                    qTable = loadedData.ToQTable();
                    //Debug.Log("Q-Table başarıyla yüklendi.");
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Q-Table yüklenirken hata oluştu: {e.Message}");
            qTable = new Dictionary<MapLocation, float[]>();
        }
    }

    [System.Serializable]
    public struct SerializableLocation
    {
        public int x;
        public int z;

        public SerializableLocation(MapLocation location)
        {
            x = location.x;
            z = location.z;
        }

        public MapLocation ToMapLocation()
        {
            return new MapLocation(x, z);
        }
    }

    [System.Serializable]
    private class SerializableState
    {
        public SerializableLocation location;
        public float[] values;

        public SerializableState(MapLocation loc, float[] vals)
        {
            location = new SerializableLocation(loc);
            values = vals;
        }
    }

    [System.Serializable]
    private class SerializableQTable
    {
        public List<SerializableState> states = new List<SerializableState>();

        public SerializableQTable(Dictionary<MapLocation, float[]> qTable)
        {
            foreach (var kvp in qTable)
            {
                states.Add(new SerializableState(kvp.Key, kvp.Value));
            }
        }

        public Dictionary<MapLocation, float[]> ToQTable()
        {
            Dictionary<MapLocation, float[]> result = new Dictionary<MapLocation, float[]>();
            foreach (var state in states)
            {
                result.Add(state.location.ToMapLocation(), state.values);
            }
            return result;
        }
    }

    void CalculateMinStepsAndVisualizePath()
    {
        ClearMarkersV5();

        if (!qTable.ContainsKey(startPoint))
        {
            Debug.LogWarning("Q-Tablosunda başlangıç durumu bulunamadı. İşlem yapılamıyor.");
            return;
        }

        MapLocation currentState = startPoint;
        List<MapLocation> path = new List<MapLocation> { currentState };
        int stepCount = 0;

        while (!currentState.Equals(goalPoint))
        {
            if (!qTable.ContainsKey(currentState))
            {
                Debug.LogWarning("Q-Tablosunda eksik durum tespit edildi. Görselleştirme tamamlanamıyor.");
                return;
            }

            int bestAction = System.Array.IndexOf(qTable[currentState], qTable[currentState].Max());
            currentState = GetNextStateV5(currentState, bestAction);
            path.Add(currentState);
            stepCount++;

            if (stepCount > 10000)
            {
                Debug.LogWarning("Adım sayısı sınırı aşıldı. Minimum adımlar başarıyla hesaplanamadı.");
                return;
            }
        }

        foreach (var location in path)
        {
            Vector3 position = new Vector3(location.x * maze.scale, 0.5f, location.z * maze.scale);
            GameObject marker = Instantiate(pathMarker, position, Quaternion.identity);
            pathMarkers.Add(marker);
        }

        Debug.Log($"Minimum adımlar hesaplandı ve yol görselleştirildi. Adım sayısı: {stepCount}");
    }

    public void ResetQTable()
    {
        qTable.Clear();
        PlayerPrefs.DeleteKey("QTableData");
        Debug.Log("Q-Table sıfırlandı.");
    }

    // Otomatik eğitim fonksiyonu
    IEnumerator AutoTrainUntilSuccess()
    {
        isAutoTraining = true;
        int attemptCount = 0;  // Toplam deneme sayısı
        bool success = false;
        int totalAutoIterations = 0;  // Toplam adım sayısı
        
        temporaryWalls.Clear();

        Debug.Log($"<color=yellow>===== BAŞLANGIÇ NOKTASI: ({startPoint.x}, {startPoint.z}) =====</color>");
        Debug.Log($"<color=yellow>===== BİTİŞ NOKTASI: ({goalPoint.x}, {goalPoint.z}) =====</color>");
        globalStartTime = Time.time;
        while (!success && attemptCount < maxTrainingAttempts)
        {
            attemptCount++;
            VisualizeLastPathV5();
            Debug.Log($"Eğitim denemesi: {attemptCount}, Toplam İterasyon: {totalAutoIterations} Sonuc2");
            
            TrainAgentV5WithoutClearingWalls();  // Her eğitimde 5 deneme hakkı var
            totalAutoIterations += iterationCount;
            
            // Başarıyı kontrol et
            MapLocation currentState = startPoint;
            int steps = 0;
            bool foundPath = false;
            
            while (!currentState.Equals(goalPoint) && steps < maze.width * maze.depth)
            {
                if (!qTable.ContainsKey(currentState))
                    break;

                int bestAction = System.Array.IndexOf(qTable[currentState], qTable[currentState].Max());
                MapLocation nextState = GetNextStateV5(currentState, bestAction);
                
                if (nextState.Equals(currentState))
                    break;
                    
                currentState = nextState;
                steps++;
                
                if (currentState.Equals(goalPoint))
                {
                    foundPath = true;
                    break;
                }
            }

            if (foundPath)
            {
                success = true;
                Debug.Log($"<color=green>BAŞARILI!</color>");
                Debug.Log($"Başlangıç: ({startPoint.x}, {startPoint.z})");
                Debug.Log($"Bitiş: ({goalPoint.x}, {goalPoint.z})");
                Debug.Log($"Toplam Eğitim Denemesi: {attemptCount}");  // Kaç eğitim denemesi yapıldı
                Debug.Log($"Son Eğitimdeki İterasyon: {iterationCount}");  // Son eğitimdeki adım sayısı
                Debug.Log($"Toplam İterasyon: {totalAutoIterations}");  // Tüm eğitimlerdeki toplam adım
                CalculateMinStepsAndVisualizePath();
            }

            yield return new WaitForSeconds(0.1f);
        }

        if (!success)
        {
            Debug.Log($"<color=red>BAŞARISIZ!</color>");
            Debug.Log($"Başlangıç: ({startPoint.x}, {startPoint.z})");
            Debug.Log($"Bitiş: ({goalPoint.x}, {goalPoint.z})");
            Debug.Log($"Toplam Eğitim Denemesi: {attemptCount} Sonuc2");
            Debug.Log($"Son Eğitimdeki İterasyon: {iterationCount} Sonuc2");
            Debug.Log($"Toplam İterasyon: {totalAutoIterations}");
        }

        isAutoTraining = false;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R) && !isTraining)
        {
            RandomizeStartAndGoalV5();
        }

        if (Input.GetKeyDown(KeyCode.T) && !isTraining)
        {
            Debug.Log("Q-Learning V5 eğitimine başlandı!");
            TrainAgentV5();
        }

        if (Input.GetKeyDown(KeyCode.O))
        {
            VisualizeBestPathV5();
        }

        if (Input.GetKeyDown(KeyCode.V))
        {
            VisualizeLastPathV5();
        }

        if (Input.GetKeyDown(KeyCode.M))
        {
            CalculateMinStepsAndVisualizePath();
        }

        if (Input.GetKeyDown(KeyCode.C))
        {
            ClearMarkersV5();
        }

        if (Input.GetKeyDown(KeyCode.X)) // X tuşu Q-Table'ı sıfırlar
        {
            ResetQTable();
            //ClearDeadEndMarkers();
            //ClearMarkersV5();
        }

        if (Input.GetKeyDown(KeyCode.F) && !isTraining && !isAutoTraining) // F tuşu otomatik eğitimi başlatır
        {
            Debug.Log("Otomatik eğitim başlatılıyor...");
            StartCoroutine(AutoTrainUntilSuccess());
        }

        if (Input.GetKeyDown(KeyCode.Escape) && isAutoTraining) // ESC tuşu otomatik eğitimi durdurur
        {
            StopAllCoroutines();
            isAutoTraining = false;
            Debug.Log("Otomatik eğitim durduruldu.");
        }

        if (Input.GetKeyDown(KeyCode.Z)) // Z tuşu çıkmaz yol işaretçilerini temizler
        {
            ClearDeadEndMarkers();
        }
    }

    private float GetDynamicLearningRate()
    {
        // Eğitim sayısı arttıkça öğrenme oranını azalt
        return Mathf.Max(minLearningRate, 
            initialLearningRate * (1f - (float)totalTrainingCount / 1000));
    }

    // Sadece R tuşuna basıldığında veya yeni başlangıç/bitiş noktaları seçildiğinde tüm işaretçileri temizle
    void ClearAllMarkers()
    {
        // Yol işaretçilerini temizle
        foreach (var marker in pathMarkers)
        {
            Destroy(marker);
        }
        pathMarkers.Clear();

        // Çıkmaz yol işaretçilerini temizle
        foreach (var marker in deadEndMarkers)
        {
            Destroy(marker);
        }
        deadEndMarkers.Clear();
    }

    // Sadece çıkmaz yol işaretçilerini temizleyen fonksiyon
    void ClearDeadEndMarkers()
    {
        foreach (var marker in deadEndMarkers)
        {
            Destroy(marker);
        }
        deadEndMarkers.Clear();
        temporaryWalls.Clear(); // Geçici duvar listesini de temizle
        Debug.Log("Çıkmaz yol işaretçileri temizlendi.");
    }

    public MapLocation GetStartPoint() { return startPoint; }
    public MapLocation GetGoalPoint() { return goalPoint; }
    public bool IsTraining() { return isTraining || isAutoTraining; }
    public void StartAutoTraining() { StartCoroutine(AutoTrainUntilSuccess()); }
} 