using UnityEngine;
using System.IO;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;

public class TerrainDataExporter : MonoBehaviour
{
    public Camera orthographicCamera;
    public Terrain terrain;
    public int resolution = 100;
    public int port = 5555;
    
    [Header("Trajectory Visualization")]
    public bool showTrajectory = true;
    public Color trajectoryColor = Color.red;
    public float trajectoryWidth = 0.5f;
    public Material trajectoryMaterial;
    
    private TcpListener server;
    private Thread serverThread;
    private bool isRunning = false;
    
    // Queue for thread-safe camera movement
    private Vector3? pendingCameraPosition = null;
    private bool shouldExportData = false;
    private bool shouldGetHighestPoint = false;
    private Vector3 highestPointResult = Vector3.zero;
    
    // Trajectory tracking
    private LineRenderer trajectoryLine;
    private List<Vector3> trajectoryPoints = new List<Vector3>();
    
    void Start()
    {
        SetupTrajectoryLine();
        StartServer();
    }
    
    void SetupTrajectoryLine()
    {
        // Create a new GameObject for the line renderer
        GameObject lineObj = new GameObject("CameraTrajectory");
        lineObj.transform.SetParent(transform);
        
        trajectoryLine = lineObj.AddComponent<LineRenderer>();
        
        // Configure line renderer
        trajectoryLine.startWidth = trajectoryWidth;
        trajectoryLine.endWidth = trajectoryWidth;
        trajectoryLine.positionCount = 0;
        
        // Set material
        if (trajectoryMaterial != null)
        {
            trajectoryLine.material = trajectoryMaterial;
        }
        else
        {
            // Create a default unlit material
            trajectoryLine.material = new Material(Shader.Find("Sprites/Default"));
        }
        
        trajectoryLine.startColor = trajectoryColor;
        trajectoryLine.endColor = trajectoryColor;
        
        // Disable to cast/receive shadows
        trajectoryLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        trajectoryLine.receiveShadows = false;
    }
    
    void Update()
    {
        // Handle camera movement on main thread
        if (pendingCameraPosition.HasValue)
        {
            orthographicCamera.transform.position = pendingCameraPosition.Value;
            
            // Add point to trajectory
            if (showTrajectory)
            {
                AddTrajectoryPoint(pendingCameraPosition.Value);
            }
            
            pendingCameraPosition = null;
            Debug.Log($"Camera moved to: {orthographicCamera.transform.position}");
        }
        
        // Handle data export on main thread
        if (shouldExportData)
        {
            ExportTerrainData();
            shouldExportData = false;
        }
        
        // Handle highest point calculation on main thread
        if (shouldGetHighestPoint)
        {
            highestPointResult = GetHighestPoint();
            shouldGetHighestPoint = false;
        }
    }
    
    void AddTrajectoryPoint(Vector3 point)
    {
        trajectoryPoints.Add(point);
        trajectoryLine.positionCount = trajectoryPoints.Count;
        trajectoryLine.SetPosition(trajectoryPoints.Count - 1, point);
    }
    
    // Public method to clear the trajectory
    public void ClearTrajectory()
    {
        trajectoryPoints.Clear();
        trajectoryLine.positionCount = 0;
    }
    
    void StartServer()
    {
        serverThread = new Thread(new ThreadStart(ServerLoop));
        serverThread.IsBackground = true;
        serverThread.Start();
        isRunning = true;
        Debug.Log($"Server started on port {port}");
    }
    
    void ServerLoop()
    {
        server = new TcpListener(IPAddress.Parse("127.0.0.1"), port);
        server.Start();
        
        while (isRunning)
        {
            try
            {
                TcpClient client = server.AcceptTcpClient();
                HandleClient(client);
            }
            catch (System.Exception e)
            {
                if (isRunning)
                    Debug.LogError("Server error: " + e.Message);
            }
        }
    }
    
    void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        byte[] buffer = new byte[1024];
        int bytesRead = stream.Read(buffer, 0, buffer.Length);
        string message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
        
        Debug.Log("Received: " + message);
        
        // Parse command
        string response = ProcessCommand(message);
        
        // Send response
        byte[] responseBytes = Encoding.UTF8.GetBytes(response);
        stream.Write(responseBytes, 0, responseBytes.Length);
        
        stream.Close();
        client.Close();
    }
    
    string ProcessCommand(string message)
    {
        try
        {
            CommandData cmd = JsonUtility.FromJson<CommandData>(message);
            
            switch (cmd.command)
            {
                case "move_camera":
                    pendingCameraPosition = new Vector3(cmd.x, cmd.y, cmd.z);
                    return "{\"status\":\"success\",\"message\":\"Camera move queued\"}";
                    
                case "move_and_export":
                    pendingCameraPosition = new Vector3(cmd.x, cmd.y, cmd.z);
                    Thread.Sleep(1);
                    shouldExportData = true;
                    return "{\"status\":\"success\",\"message\":\"Move and export queued\"}";
                
                case "clear_trajectory":
                    ClearTrajectory();
                    return "{\"status\":\"success\",\"message\":\"Trajectory cleared\"}";

                default:
                    return "{\"status\":\"error\",\"message\":\"Unknown command\"}";
            }
        }
        catch (System.Exception e)
        {
            return "{\"status\":\"error\",\"message\":\"" + e.Message + "\"}";
        }
    }
    
    Vector3 GetHighestPoint()
    {
        if (terrain == null)
        {
            Debug.LogError("Terrain reference is null");
            return Vector3.zero;
        }
        
        UnityEngine.TerrainData terrainData = terrain.terrainData;
        Vector3 terrainPos = terrain.transform.position;
        
        int heightmapWidth = terrainData.heightmapResolution;
        int heightmapHeight = terrainData.heightmapResolution;
        
        float maxHeight = float.MinValue;
        int maxHeightIndexX = 0;
        int maxHeightIndexZ = 0;
        
        for (int z = 0; z < heightmapHeight; z++)
        {
            for (int x = 0; x < heightmapWidth; x++)
            {
                float height = terrainData.GetHeight(x, z);
                
                if (height > maxHeight)
                {
                    maxHeight = height;
                    maxHeightIndexX = x;
                    maxHeightIndexZ = z;
                }
            }
        }
        
        float worldX = terrainPos.x + (maxHeightIndexX / (float)(heightmapWidth - 1)) * terrainData.size.x;
        float worldZ = terrainPos.z + (maxHeightIndexZ / (float)(heightmapHeight - 1)) * terrainData.size.z;
        float worldY = terrainPos.y + maxHeight;
        
        return new Vector3(worldX, worldY, worldZ);
    }
    
    public void ExportTerrainData()
    {
        float camHeight = orthographicCamera.orthographicSize;
        float camWidth = camHeight * orthographicCamera.aspect;
        Vector3 camPos = orthographicCamera.transform.position;
        
        float minX = camPos.x - camWidth;
        float maxX = camPos.x + camWidth;
        float minZ = camPos.z - camHeight;
        float maxZ = camPos.z + camHeight;
        
        float[] xCoords = new float[resolution];
        float[] yCoords = new float[resolution];
        float[][] zHeights = new float[resolution][];
        
        for (int j = 0; j < resolution; j++)
        {
            xCoords[j] = Mathf.Lerp(minX, maxX, j / (float)(resolution - 1));
        }
        
        for (int i = 0; i < resolution; i++)
        {
            yCoords[i] = Mathf.Lerp(minZ, maxZ, i / (float)(resolution - 1));
        }
        
        for (int i = 0; i < resolution; i++)
        {
            zHeights[i] = new float[resolution];
            
            for (int j = 0; j < resolution; j++)
            {
                float x = xCoords[j];
                float z = yCoords[i];
                zHeights[i][j] = SampleTerrainHeight(x, z);
            }
        }
        
        ExportToJSON(xCoords, yCoords, zHeights);
    }
    
    float SampleTerrainHeight(float worldX, float worldZ)
    {
        if (terrain == null)
        {
            RaycastHit hit;
            Vector3 rayOrigin = new Vector3(worldX, orthographicCamera.transform.position.y + 100, worldZ);
            
            if (Physics.Raycast(rayOrigin, Vector3.down, out hit, Mathf.Infinity))
            {
                return hit.point.y;
            }
            return 0f;
        }
        else
        {
            return terrain.SampleHeight(new Vector3(worldX, 0, worldZ));
        }
    }
    
    void ExportToJSON(float[] xCoords, float[] yCoords, float[][] zHeights)
    {
        StringBuilder json = new StringBuilder();
        json.Append("{");
        
        json.Append("\"x\":[");
        for (int i = 0; i < xCoords.Length; i++)
        {
            json.Append(xCoords[i].ToString("F6"));
            if (i < xCoords.Length - 1) json.Append(",");
        }
        json.Append("],");
        
        json.Append("\"y\":[");
        for (int i = 0; i < yCoords.Length; i++)
        {
            json.Append(yCoords[i].ToString("F6"));
            if (i < yCoords.Length - 1) json.Append(",");
        }
        json.Append("],");
        
        json.Append("\"z\":[");
        for (int i = 0; i < zHeights.Length; i++)
        {
            json.Append("[");
            for (int j = 0; j < zHeights[i].Length; j++)
            {
                json.Append(zHeights[i][j].ToString("F6"));
                if (j < zHeights[i].Length - 1) json.Append(",");
            }
            json.Append("]");
            if (i < zHeights.Length - 1) json.Append(",");
        }
        json.Append("]");
        
        json.Append("}");
        
        string filePath = Path.Combine(Application.dataPath, "terrain_data.json");
        File.WriteAllText(filePath, json.ToString());
        Debug.Log("Terrain data exported to: " + filePath);
    }
    
    void OnApplicationQuit()
    {
        isRunning = false;
        if (server != null)
            server.Stop();
        if (serverThread != null)
            serverThread.Abort();
    }
}

[System.Serializable]
public class CommandData
{
    public string command;
    public float x;
    public float y;
    public float z;
}