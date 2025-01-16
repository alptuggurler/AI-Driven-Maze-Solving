using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;


public class PathMarker
{
    public MapLocation location;
    public float G;
    public float H;
    public float F;
    public GameObject marker;
    public PathMarker parent;

    

    public PathMarker(MapLocation l, float g, float h, float f, GameObject marker, PathMarker p)
    {
        location = l;
        G = g;
        H = h;
        F = f;
        this.marker = marker;
        parent = p;
    }
    public override bool Equals(object obj)
    {
        if ((obj == null) || !this.GetType().Equals(obj.GetType()))
        {
            return false;
        }
        else
        {
            return location.Equals(((PathMarker)obj).location);
        }
    }
    public override int GetHashCode()
    {
        return 0;
    }
}

public class FindPathAStar_sc : MonoBehaviour
{

    public Maze maze;
    public Material closedMaterial;
    public Material openMaterial;
    List<PathMarker> open = new List<PathMarker>();
    List<PathMarker> closed = new List<PathMarker>();
    public GameObject start;
    public GameObject end;
    public GameObject pathP;

    public PathMarker startNode;
    public PathMarker goalNode;
    public PathMarker lastPos;
    bool done = false;

    private int totalSteps = 0;
    private float pathDistance = 0f;

    private bool isWaitingForInput = false;
    private bool isWaitingForStartInput = false;
    private string inputBuffer = "";
    private float startTime;
    private Stopwatch stopwatch = new Stopwatch();

    void RemoveAllMarkers()
    {
        GameObject[] markers = GameObject.FindGameObjectsWithTag("marker");
        foreach (GameObject m in markers)
            Destroy(m);
    }

    void BeginSearch()
    {
        done = false;
        RemoveAllMarkers();
        List<MapLocation> locations = new List<MapLocation>();
        for (int z = 1; z < maze.depth - 1; z++)
            for (int x = 1; x < maze.depth - 1; x++)
            {
                if (maze.map[x, z] != 1) //0: empty space, 1: wall
                    locations.Add(new MapLocation(x, z));
            }
        locations.Shuffle();
        //0 for G, H, and F values. Null for the parent.
        Vector3 startLocation = new Vector3(locations[0].x * maze.scale, 0, locations[0].z * maze.scale);
        startNode = new PathMarker(new MapLocation(locations[0].x, locations[0].z), 0, 0, 0,
        Instantiate(start, startLocation, Quaternion.identity), null);
        Vector3 goalLocation = new Vector3(locations[1].x * maze.scale, 0, locations[1].z * maze.scale);
        goalNode = new PathMarker(new MapLocation(locations[1].x, locations[1].z), 0, 0, 0,
        Instantiate(end, goalLocation, Quaternion.identity), null);


        open.Clear();
        closed.Clear();
        open.Add(startNode);
        lastPos = startNode;

    }

    void Search(PathMarker thisNode)
    {
        if (thisNode == null) return;

        if (thisNode.Equals(goalNode))
        {
            done = true; return;
        }

        //The neighbors are horizontal and vertical but we can also add diagonals as well. You can check the Maze script.
        foreach (MapLocation dir in maze.directions)
        {
            MapLocation neighbor = dir + thisNode.location;
            if (maze.map[neighbor.x, neighbor.z] == 1) continue; //skip walls
            if (neighbor.x < 1 || neighbor.x >= maze.width || neighbor.z < 1 || neighbor.z >= maze.depth) continue;
            if (IsClosed(neighbor)) continue;

            float G = Vector2.Distance(thisNode.location.ToVector(), neighbor.ToVector()) + thisNode.G;
            float H = Vector2.Distance(neighbor.ToVector(), goalNode.location.ToVector());
            float F = G + H;
            GameObject pathBlock = Instantiate(pathP, new Vector3(neighbor.x * maze.scale, 0, neighbor.z * maze.scale), Quaternion.identity);
            TextMesh[] values = pathBlock.GetComponentsInChildren<TextMesh>();
            //In the prefab, G is attached first, then H, and then F. You can check the prefab
            values[0].text = "G: " + G.ToString("0.00");
            values[1].text = "H: " + G.ToString("0.00");
            values[2].text = "F: " + G.ToString("0.00");
            if (!UpdateMarker(neighbor, G, H, F, thisNode))
                open.Add(new PathMarker(neighbor, G, H, F, pathBlock, thisNode));
        }
        //Select the one from the open with the smallest F value
        //If multiple F values, select the one with the smallest H value
        open = open.OrderBy(p => p.F).ThenBy(n => n.H).ToList<PathMarker>();
        PathMarker pm = (PathMarker)open.ElementAt(0);
        closed.Add(pm);
        open.RemoveAt(0);
        pm.marker.GetComponent<Renderer>().material = closedMaterial;
        lastPos = pm;
    }
    bool IsClosed(MapLocation marker)
    {
        foreach (PathMarker p in closed)
        {
            if (p.location.Equals(marker)) return true;
        }
        return false;
    }
    bool UpdateMarker(MapLocation pos, float g, float h, float f, PathMarker prt)
    {
        foreach (PathMarker p in open)
        {
            if (p.location.Equals(pos))
            {
                p.G = g;
                p.H = h;
                p.F = f;
                p.parent = prt;
                return true;
            }
        }
        return false;
    }

    void GetPath()
    {
        RemoveAllMarkers();
        PathMarker begin = lastPos;
        while (!startNode.Equals(begin) && begin != null)
        {
            Instantiate(pathP, new Vector3(begin.location.x * maze.scale, 0, begin.location.z * maze.scale), Quaternion.identity);
            begin = begin.parent;
        }
        Instantiate(pathP, new Vector3(startNode.location.x * maze.scale, 0, startNode.location.z * maze.scale), Quaternion.identity);
    }

    public void RunCompleteAStar()
    {
        // Stopwatch'ı başlat
        stopwatch.Reset();
        stopwatch.Start();
        UnityEngine.Debug.Log($"A* Çözümü başlatıldı.");
        
        // Mevcut markerları temizle ama başlangıç ve bitiş noktalarını koru
        GameObject[] markers = GameObject.FindGameObjectsWithTag("marker");
        foreach (GameObject m in markers)
        {
            if (m != startNode?.marker && m != goalNode?.marker)
                Destroy(m);
        }

        // Listeleri temizle
        open.Clear();
        closed.Clear();
        
        // Başlangıç noktasını ekle
        if (startNode != null)
        {
            open.Add(startNode);
            lastPos = startNode;
            done = false;
            
            // A* algoritmasını çözüm bulana kadar çalıştır
            while (!done)
            {
                Search(lastPos);
                totalSteps++;
            }
            
            // Çözüm bulundu, yolu göster ve mesafeyi hesapla
            if (done)
            {
                pathDistance = 0;
                PathMarker begin = lastPos;
                List<PathMarker> path = new List<PathMarker>();
                
                // Yolu ve mesafeyi hesapla
                while (!startNode.Equals(begin) && begin != null)
                {
                    path.Add(begin);
                    if (begin.parent != null)
                    {
                        pathDistance += Vector2.Distance(
                            begin.location.ToVector(),
                            begin.parent.location.ToVector()
                        );
                    }
                    begin = begin.parent;
                }
                
                // Başlangıç noktasını ekle
                path.Add(startNode);
                
                // Yolu görselleştir (başlangıç ve bitiş markerlarını koruyarak)
                foreach (var marker in path)
                {
                    if (marker != startNode && marker != goalNode)
                    {
                        Instantiate(pathP, 
                            new Vector3(marker.location.x * maze.scale, 0, marker.location.z * maze.scale),
                            Quaternion.identity
                        );
                    }
                }
                
                // Stopwatch'ı durdur ve süreyi hesapla
                stopwatch.Stop();
                double totalTimeMs = stopwatch.Elapsed.TotalMilliseconds;
                UnityEngine.Debug.Log($"<color=#FFD700><b>Geçen Süre: {totalTimeMs:F3} milisaniye</b></color>");
                UnityEngine.Debug.Log($"<color=#FFD700><b>Geçen Süre: {totalTimeMs/1000:F6} saniye Sonuc2</b></color>");
                UnityEngine.Debug.Log($"<color=green>A* Çözüm Bulundu!</color>");
                UnityEngine.Debug.Log($"Başlangıç: ({startNode.location.x}, {startNode.location.z})");
                UnityEngine.Debug.Log($"Hedef: ({goalNode.location.x}, {goalNode.location.z})");
                UnityEngine.Debug.Log($"Toplam Adım Sayısı: {totalSteps} Sonuc2");
                UnityEngine.Debug.Log($"Yol Uzunluğu: {pathDistance:F2} birim Sonuc2");
                UnityEngine.Debug.Log($"Yoldaki Nokta Sayısı: {path.Count} Sonuc2");
            }
        }
        else
        {
            UnityEngine.Debug.LogWarning("Önce P tuşu ile başlangıç ve bitiş noktalarını belirleyin!");
        }
    }

    void SetCustomPoints()
    {
        if (!isWaitingForInput)
        {
            isWaitingForInput = true;
            isWaitingForStartInput = true;
            inputBuffer = "";
            UnityEngine.Debug.Log("Başlangıç noktası için x,z koordinatlarını girin (örn: 3,4):");
        }
    }

    // Sınıf seviyesinde bir Coroutine tanımlayalım
    IEnumerator StartAStarWithDelay()
    {
        UnityEngine.Debug.Log("A* algoritması 1 saniye içinde başlayacak...");
        yield return new WaitForSeconds(1);
        RunCompleteAStar();
    }

    void Update()
    {
        if (isWaitingForInput)
        {
            foreach (char c in Input.inputString)
            {
                if (c == '\b') // Backspace
                {
                    if (inputBuffer.Length > 0)
                    {
                        inputBuffer = inputBuffer.Substring(0, inputBuffer.Length - 1);
                    }
                }
                else if (c == '\n' || c == '\r') // Enter
                {
                    ProcessInput();
                }
                else
                {
                    inputBuffer += c;
                }
            }
        }
        else
        {
            // Normal tuş kontrolleri
            if (Input.GetKeyDown(KeyCode.P)) BeginSearch();
            if (Input.GetKeyDown(KeyCode.C) && !done) Search(lastPos);
            if (Input.GetKeyDown(KeyCode.M)) GetPath();
            if (Input.GetKeyDown(KeyCode.K))
            {
                UnityEngine.Debug.Log($"Geçen Süre: {Time.time} saniye");
                totalSteps = 0;
                pathDistance = 0;
                startTime = Time.time;
                StartCoroutine(StartAStarWithDelay());
            }
            if (Input.GetKeyDown(KeyCode.I)) // I tuşu için özel nokta girişi
            {
                SetCustomPoints();
            }
        }
    }

    void ProcessInput()
    {
        string[] coordinates = inputBuffer.Split(',');
        if (coordinates.Length == 2 && 
            int.TryParse(coordinates[0], out int x) && 
            int.TryParse(coordinates[1], out int z))
        {
            // Koordinatların geçerli olup olmadığını kontrol et
            if (x >= 1 && x < maze.width - 1 && z >= 1 && z < maze.depth - 1 && maze.map[x,z] != 1)
            {
                if (isWaitingForStartInput)
                {
                    // Başlangıç noktasını ayarla
                    RemoveAllMarkers();
                    Vector3 startLocation = new Vector3(x * maze.scale, 0, z * maze.scale);
                    startNode = new PathMarker(new MapLocation(x, z), 0, 0, 0,
                        Instantiate(start, startLocation, Quaternion.identity), null);
                    
                    // Bitiş noktası için input bekle
                    isWaitingForStartInput = false;
                    inputBuffer = "";
                    UnityEngine.Debug.Log("Bitiş noktası için x,z koordinatlarını girin (örn: 3,4):");
                    return;
                }
                else
                {
                    // Bitiş noktasını ayarla
                    Vector3 goalLocation = new Vector3(x * maze.scale, 0, z * maze.scale);
                    goalNode = new PathMarker(new MapLocation(x, z), 0, 0, 0,
                        Instantiate(end, goalLocation, Quaternion.identity), null);
                    
                    // Listeleri temizle ve başlangıç noktasını ekle
                    open.Clear();
                    closed.Clear();
                    open.Add(startNode);
                    lastPos = startNode;
                    done = false;

                    UnityEngine.Debug.Log($"Noktalar ayarlandı - Başlangıç: ({startNode.location.x},{startNode.location.z}), Bitiş: ({x},{z})");
                }
            }
            else
            {
                UnityEngine.Debug.LogWarning("Geçersiz koordinatlar! Lütfen duvar olmayan ve sınırlar içinde bir nokta girin.");
                inputBuffer = "";
                return;
            }
        }
        else
        {
            UnityEngine.Debug.LogWarning("Hatalı format! Lütfen x,z şeklinde girin (örn: 3,4)");
            inputBuffer = "";
            return;
        }

        isWaitingForInput = false;
        inputBuffer = "";
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame

}
