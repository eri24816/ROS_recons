using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;


[ExecuteAlways]
public class Builder : MonoBehaviour
{
    RosMessageTypes.Nav.OccupancyGridMsg mapMsg;

    [SerializeField]
    Mesh mesh;
    public int w,h;
    public float[,] map;
    ROSConnection ros;
    // Start is called before the first frame update
    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe< RosMessageTypes.Nav.MapMetaDataMsg> ("map_metadata", ReceiveMapMetaData);
        ros.Subscribe<RosMessageTypes.Nav.OccupancyGridMsg>("map", ReceiveOccupancyGrid);

        GetComponent<MeshFilter>().mesh = mesh = new Mesh();
        ht = new HoughTransform();
        //GenerateMap();
        //BuildPixel(map);
        //BuildWalkAround(map);
    }

    void ReceiveMapMetaData(RosMessageTypes.Nav.MapMetaDataMsg msg)
    {

    }
    void ReceiveOccupancyGrid(RosMessageTypes.Nav.OccupancyGridMsg msg)
    {
        mapMsg = msg;
        print(msg);
    }

    public void ReadMap()
    {
        float[,] rawMap = new float[mapMsg.info.width, mapMsg.info.height];
        for (int i = 0; i < mapMsg.info.width; i++)
        {
            for (int j = 0; j < mapMsg.info.height; j++)
            {
                int v = mapMsg.data[i + j * mapMsg.info.width];
                if (v == 100) rawMap[i, j] = 1;
            }
        }
        map = FieldOperations.DownScale(rawMap, 4);

        for (int i = 0; i < map.GetLength(0); i++)
        {
            for (int j = 0; j < map.GetLength(1); j++)
            {
                map[i, j] = map[i, j] > 0.001 ? 1 : 0;
            }
        }

        w =map.GetLength(0);
        h=map.GetLength(1);
        //PrintMap(map);
    }

    public void GenerateMap()
    {
        map = new float[w, h];

        for (int x = 0; x < map.GetLength(0) - 0; x++)
        {
            for (int y = 0; y < map.GetLength(1) -0; y++)
            {
                if (x == y || x == y + 1)
                {
                    map[x, y] = 1;
                }
            }
        }

        for (int x = 1; x < map.GetLength(0) - 1; x++)
        {
            for (int y = 1; y < map.GetLength(1) - 1; y++)
            {
                if ((x == 8))
                {
                    map[x, y] = 1;
                }
            }
        }
        map[21, 3] = 1;
        map[20, 3] = 1;
        map[21, 5] = 1;
        PrintMap(map);
    }

    public int startPointGap = 1;
    public float stayOnIntensity = 0.3f;
    public float snap = 0.6f;
    public float stride = 0.3f;
    public float fixSpeed = 0.5f;
    public float mergeAngle = 15f;
    public void BuildWalkAround(float[,] map)
    {
        int pad = 4;
        map = FieldOperations.AddPadding(map, pad);
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        Dictionary<Vector2Int, System.Tuple<Vector2, float>> vis = new Dictionary<Vector2Int, System.Tuple<Vector2, float>>();
        var gradient = FieldOperations.Gradient(FieldOperations.Blur(map, 3));

        for (int startX = 0; startX < w; startX += startPointGap)
        {
            for (int startY = 0; startY < h; startY += startPointGap)
            {
                foreach (float dir in new float[] { 1, -1 })
                {
                    var points = WalkAround(gradient, map, new Vector2(startX+0.5f, startY+0.5f), vis, dir);
                    if (points.Count == 0) continue;

                    bool loop = false;
                    if (points[0] == points[points.Count - 1]) { points.RemoveAt(points.Count - 1); loop = true; }

                    // Optimize face count
                    int maxIter = points.Count * 3;
                    for (int i = 0; i < maxIter; i++)
                    {
                        Vector3 a = points[i % points.Count], b = points[(i + 1) % points.Count], c = points[(i + 2) % points.Count];
                        if (Vector2.Angle(b - a, c - b) < mergeAngle)
                        {
                            points.RemoveAt((i + 1) % points.Count);
                            i--;
                        }
                    }
                    if (points.Count == 0) continue;

                    // Remove padding translation
                    for (int i = 0; i < points.Count; i++)
                    {
                        points[i] -= pad * Vector2.one;
                    }

                    // Build
                    for (int i = 0; i < points.Count - 1; i++)
                    {
                        BuildWall2(vertices, triangles, V22V3(points[i]), V22V3(points[i + 1]), 4);
                    }
                    if (loop)
                    {
                        BuildWall2(vertices, triangles, V22V3(points[points.Count - 1]), V22V3(points[0]), 4);
                    }
                }
            }
        }
        mesh.Clear();
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals(0);
        mesh.RecalculateBounds();
    }
    int iStart = 0;
    List<Vector2> WalkAround(Vector2[,] grad, float[,] map, Vector2 startPoint, Dictionary<Vector2Int, System.Tuple<Vector2, float>> vis = null,float dir=-1)
    {
        //float stayOnIntensity = 0.3f;
        //float snap = 0.6f;
        System.Func<Vector2, Vector2Int> Snap = (Vector2 v) => new Vector2Int(Mathf.RoundToInt(v.x / snap) , Mathf.RoundToInt(v.y / snap));
        List<Vector2> result = new List<Vector2>();
        if (vis is null)
        {
            vis = new Dictionary<Vector2Int, System.Tuple<Vector2, float>>();
        }
        //float stride = .3f;
        Vector2 pos = startPoint;
        float m=-1;
        if (GetValueAt(map, pos.x, pos.y) < 0.01) return result;

        for (int i = 0; i < 10; i++)
        {
            Vector2 g = GetValueAt(grad, pos.x, pos.y);
            if (g.magnitude == 0)
            {
                return result;
                //g = Rotate( Vector2.right,Random.Range(0,6.283f));
            }
            g.Normalize();

            m = GetValueAt(map, pos.x, pos.y);
            if (m < stayOnIntensity)
            {
                pos += g * (.03f + stayOnIntensity - m) * fixSpeed;
            }
            else if (m > stayOnIntensity + .1f)
            {
                pos -= g * (.03f + m - (stayOnIntensity + .1f)) * fixSpeed;
            }
            
            Vector2 vel = Rotate(g, dir*Mathf.PI / 2) * stride*0.3f;
            //pos += vel;
        }
        

        iStart += 10;
        for (int i = iStart; i < iStart+ 300; i++)
        {
            Vector2 g = GetValueAt(grad, pos.x, pos.y);
            g.Normalize();

            if (g == Vector2.zero)
            {
                return result;
            }
            Vector2 vel = Rotate(g, dir*Mathf.PI / 2) * stride;

            System.Tuple<Vector2, float> visPosTime;
            if (vis.TryGetValue(Snap(pos + 0.33f * vel), out visPosTime) || vis.TryGetValue(Snap(pos + 0.67f * vel), out visPosTime) || vis.TryGetValue(Snap(pos + vel), out visPosTime))
            {
                if (Mathf.Abs(visPosTime.Item2 - i) >= 3)
                {
                    result.Add(visPosTime.Item1);
                    //print($"i: {i-iStart}");
                    return result;
                }
            }

            string debug = "";
            pos += vel;
            for (int t = 0; t <= 30; t++)
            {
                if (t == 30) { return result; }

                m = GetValueAt(map, pos.x, pos.y);
                g = GetValueAt(grad, pos.x, pos.y);
                if (g.magnitude == 0)
                {
                    print(debug);
                    return result;
                }
                g.Normalize();
                if (m < stayOnIntensity)
                {
                    pos += g * (.03f+stayOnIntensity - m) * fixSpeed;
                }
                else if (m > stayOnIntensity + .1f)
                {
                    pos -= g * (.03f + m - (stayOnIntensity + .1f)) * fixSpeed;
                }
                else
                {
                    break;
                }
                debug += m.ToString("0.00") + " ";
            }
            print(debug);

            if (i >= 0)
            {
                result.Add(pos);
                vis[Snap(pos)] = new System.Tuple<Vector2, float>(pos, i);
            }
        }
        return result;
    }
    Vector2 Rotate(Vector2 v,float t)
    {
        return new Vector2(v.x*Mathf.Cos(t)+v.y*Mathf.Sin(t),v.x*-Mathf.Sin(t)+v.y* Mathf.Cos(t));
    }
    enum Interpolate {nearest,bilinear}
    Vector2 GetValueAt(Vector2[,] m,float x,float y,Interpolate interpolate = Interpolate.bilinear)
    {
        if(interpolate == Interpolate.nearest)
        {
            return GetValueAt(m, (int)(x + .5f), (int)(y + .5f), out Vector2 v) ? v : Vector2.zero;
        }
        if (interpolate == Interpolate.bilinear)
        {
            Vector2 result = new Vector2();
            float fsum = 0;
            foreach (var t in new int[][] {
            new int[] { Mathf.FloorToInt(x), Mathf.FloorToInt(y) },
            new int[] { Mathf.FloorToInt(x)+1, Mathf.FloorToInt(y) },
            new int[] { Mathf.FloorToInt(x), Mathf.FloorToInt(y)+1 },
            new int[] { Mathf.FloorToInt(x)+1, Mathf.FloorToInt(y)+1 },
        })
            {
                if (GetValueAt(m, t[0], t[1], out Vector2 res))
                {
                    float f = (1 - Mathf.Abs(t[0] - x)) * (1 - Mathf.Abs(t[1] - y));
                    fsum += f;
                    result += res * f;
                }
            }
            result /= fsum;
            return result;
        }
        return new Vector2(0,0);
    }
    float GetValueAt(float[,] m, float x, float y, Interpolate interpolate = Interpolate.bilinear)
    {
        if (interpolate == Interpolate.nearest)
        {
            return GetValueAt(m, (int)(x + .5f), (int)(y + .5f), out float r) ? r : 0;
        }
        if (interpolate == Interpolate.bilinear)
        {
            float result = 0;
            float fsum = 0;
            foreach (var t in new int[][] {
            new int[] { Mathf.FloorToInt(x), Mathf.FloorToInt(y) },
            new int[] { Mathf.FloorToInt(x)+1, Mathf.FloorToInt(y) },
            new int[] { Mathf.FloorToInt(x), Mathf.FloorToInt(y)+1 },
            new int[] { Mathf.FloorToInt(x)+1, Mathf.FloorToInt(y)+1 },
        })
            {
                if (GetValueAt(m, t[0], t[1], out float res))
                {
                    float f = (1 - Mathf.Abs(t[0] - x)) * (1 - Mathf.Abs(t[1] - y));
                    fsum += f;
                    result += res * f;
                }
            }
            result /= fsum;
            return result;
        }
        return 0;
    }
    bool GetValueAt<T>(T[,] m,int x,int y,out T res)
    {
        res = default;
        if (x >= 0 && y >= 0 && x < m.GetLength(0) && y < m.GetLength(1)) { res = m[x, y]; return true; }
        else { return false; }
    }

    public float height=3;
    public void BuildPixel(float[,] map){
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
                if(map[x,y]>0.01){

                    int start_idx = vertices.Count;
                    var a = new int[]{
                        0,1,4, 1,5,4, 1,3,5, 3,7,5, 3,2,7, 2,6,7, 2,0,6, 0,4,6, 0,3,1, 2,3,0

                    }; 
                    
                    for(int i=0;i<a.Length;i++)a[i]+=start_idx;
                    triangles.AddRange(a);

                    vertices.AddRange(new Vector3[]{

                        new Vector3(x-.5f,height,y-.5f),
                        new Vector3(x+.5f,height,y-.5f),
                        new Vector3(x-.5f,height,y+.5f),
                        new Vector3(x+.5f,height,y+.5f),
                        new Vector3(x-.5f,0,y-.5f),
                        new Vector3(x+.5f,0,y-.5f),
                        new Vector3(x-.5f,0,y+.5f),
                        new Vector3(x+.5f,0,y+.5f),
                    });
                                   
                }

            }
        }
        mesh.Clear();
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals(0); 
        mesh.RecalculateBounds();


    }

    void PrintMap(float[,] m)
    {
        string debug = "";
        for (int x = 0; x < m.GetLength(0);x++)
        {
            for (int y = 0; y < m.GetLength(1); y++)
            {
                debug += m[x, y].ToString("0.00") + " ";
            }
            debug += "\n";
        }

        print(debug);
    }
    void PrintMap(Vector2[,] m)
    {
        string debug = "";
        for (int x = 0; x < m.GetLength(0); x++)
        {
            for (int y = 0; y < m.GetLength(1); y++)
            {
                debug += m[x, y].ToString() + " ";
            }
            debug += "\n";
        }

        print(debug);
    }
    HoughTransform ht;
    void BuildHT(float[,] map)
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        ht.Apply(map,50,50);

        int threshold = 10;

        int cnt = 0;
        for (int ρIdx = 0; ρIdx < ht.ρResolution; ρIdx++)
        {
            for (int θIdx = 0; θIdx < ht.θResolution; θIdx++)
            {
                if (ht.θρ[θIdx, ρIdx] >= threshold)
                    cnt++;
            }
        }
        if (cnt > 100)
        {
            print($"Too many walls{cnt}");
            return;
        }
        //PrintMap(ht.θρ);
        for (int ρIdx = 0; ρIdx < ht.ρResolution; ρIdx++)
        {
            for (int θIdx = 0; θIdx < ht.θResolution; θIdx++)
            {
                if (ht.θρ[θIdx, ρIdx] < threshold) continue;
                Ray ray = ht.θρ2Ray(ht.Idx2θ(θIdx),ht.Idx2ρ(ρIdx));
                //Debug.DrawLine(ray.origin - ray.direction*10, ray.origin + ray.direction*10,Color.red,1000);
            }
        }
        /*
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals(0);
        mesh.RecalculateBounds();*/
    }

    void BuildWall2(List<Vector3> v, List<int> t, Vector3 a, Vector3 b, float height)
    {
        BuildWall(v, t, a, b, height);
        BuildWall(v, t, b, a, height);
    }
    void BuildWall(List<Vector3> v , List<int> t, Vector3 a, Vector3 b, float height)
    {
        int i = v.Count; 
        v.AddRange(new[] { a,b,a+Vector3.up*height,b+Vector3.up*height });
        t.AddRange(new[] {i,i+1,i+2,i+1,i+3,i+2});
        Debug.DrawLine(a,b, Color.red, 2);
    }

    class HoughTransform
    {

        public int W { get; private set; }
        public int H { get; private set; }
        public int θResolution { get; private set; }
        public int ρResolution { get; private set; }
        public float[,] θρ;
        public void Apply (float[,] xy,int θResolution = 10, int ρResolution=-1)
        {
            W = xy.GetLength(0);
            H = xy.GetLength(1);
            if (ρResolution == -1)
                ρResolution = Mathf.Max(W, H);

            this.θResolution = θResolution;
            this.ρResolution = ρResolution;
            
            θρ = new float[θResolution, ρResolution];

            for(int i = 0; i < W; i++)
            {
                for(int j = 0; j < H; j++)
                {
                    if (xy[i, j] == 0) continue;
                    float x = i - ((W - 1) / 2f);
                    float y = j - ((H - 1) / 2f);
                    for (int θIdx=0; θIdx < θResolution; θIdx++)
                    {
                        float θ = Idx2θ(θIdx);
                        float ρ = x * Mathf.Cos(θ) + y * Mathf.Sin(θ);
                        int ρIdx = ρ2Idx(ρ);


                        θρ[θIdx, ρIdx] += xy[i, j];
                    }
                }
            }
        }
        
        public float Idx2θ(int i)
        {
            return Mathf.Lerp(0, Mathf.PI, i / (float)θResolution);
        }
        public float Idx2ρ(int i)
        {
            return (i/(float)(ρResolution-1)*2-1)*Mathf.Max(W,H);
        }
        public int ρ2Idx(float ρ)
        {
            return (int)Mathf.Clamp(((ρ / Mathf.Max(W, H) + 1) / 2 * (float)(ρResolution - 1)),0,ρResolution - 1);
        }
        public Ray θρ2Ray(float θ,float ρ)
        {
            Vector3 origin = V22V3( ρ * new Vector2(Mathf.Cos(θ), Mathf.Sin(θ)) + ((W - 1) / 2f) * Vector2.one);
            Vector3 direction = V22V3( new Vector2(-Mathf.Sin(θ), Mathf.Cos(θ))); 
            Ray ray = new Ray(origin, direction);
            return ray; 
        }
        
    }
    public static Vector3 V22V3(Vector2 v)
    {
        return new Vector3(v.x, 0, v.y);
    }
}


