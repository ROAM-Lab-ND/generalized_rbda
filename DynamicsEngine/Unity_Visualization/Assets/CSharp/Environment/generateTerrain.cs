using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class generateTerrain : MonoBehaviour
{
    public const float maxViewDst = 50; // view distance of robot
    public Transform viewer; // terrain generated about this POV
    public GameObject cameraSelection; // POVs
    public static Vector2 viewerPosition; // x-z posn of viewer
    private static Material tileShader;

    // storing posn and chunk data of all generated tiles
    Dictionary<Vector2,TerrainChunk> terrainChunkDict = new Dictionary<Vector2,TerrainChunk>();
    List<TerrainChunk> terrainChunksVisibleLastUpdate = new List<TerrainChunk>();


    private int currentCamera; // selected camera mode
    int chunkSize; // size of chunk
    int chunksVisibleInView; // number of chunks visible in POV

    // inintializes chunk parameters, finds camera selector GameObject
    void Start(){
      chunkSize = 50; // arbitrary size, "big enough"
      chunksVisibleInView = Mathf.RoundToInt(maxViewDst/chunkSize);
      cameraSelection = GameObject.Find("Cameras");
      tileShader = (Material)Resources.Load("tile_ground",typeof(Material));
    }

    // finds current camera state, assigns viewer accordingly, updates chunks in that posn
    void Update(){
      currentCamera = cameraSelection.GetComponent<CameraSelection>().cameraState;
      switch(currentCamera){
        case 0:
          viewer = cameraSelection.transform.GetChild(0);
          break;
        case 1:
          viewer = cameraSelection.transform.GetChild(1);
          break;
        case 2:
          viewer = cameraSelection.transform.GetChild(2);
          break;
      }
      viewerPosition = new Vector2(viewer.position.x,viewer.position.z);
      UpdateVisibleChunks();
    }


    void UpdateVisibleChunks(){
      // turns off all chunks out of visible range
      for(int i = 0;i<terrainChunksVisibleLastUpdate.Count;i++){
        terrainChunksVisibleLastUpdate[i].SetVisible(false);
      }
      terrainChunksVisibleLastUpdate.Clear();

      int currentChunkCoordX = Mathf.RoundToInt(viewerPosition.x/chunkSize);
      int currentChunkCoordY = Mathf.RoundToInt(viewerPosition.y/chunkSize);

      // iterates through tile positions, adds to dictionary if not already known
      // generates new tile if one doesn't exist at the position
      for(int yOffset = -chunksVisibleInView; yOffset<= chunksVisibleInView; yOffset++){
        for(int xOffset = -chunksVisibleInView; xOffset<= chunksVisibleInView; xOffset++){
          Vector2 viewedChunkCoord = new Vector2(currentChunkCoordX+xOffset,currentChunkCoordY+yOffset);

          if (terrainChunkDict.ContainsKey(viewedChunkCoord)){
            terrainChunkDict[viewedChunkCoord].UpdateTerrainChunk();
            if(terrainChunkDict[viewedChunkCoord].isVisible()){
              terrainChunksVisibleLastUpdate.Add(terrainChunkDict[viewedChunkCoord]);
            }
            }else{
            terrainChunkDict.Add(viewedChunkCoord,new TerrainChunk(viewedChunkCoord,chunkSize, transform));
          }
        }
      }
    }

    public class TerrainChunk{

      GameObject meshObject;
      Vector2 position;
      Bounds bounds;

      // chunks set as planes, childed under terrainGenerator GameObject
      public TerrainChunk(Vector2 coord,int size, Transform parent){
        position = coord*size;
        bounds = new Bounds(position,Vector2.one*size);
        Vector3 positionV3 = new Vector3(position.x,0.01f,position.y);


        meshObject = GameObject.CreatePrimitive(PrimitiveType.Plane);
        meshObject.GetComponent<Renderer>().material = tileShader; // adds custom shader
        meshObject.transform.position = positionV3;
        meshObject.transform.localScale = Vector3.one*size/10f;
        meshObject.transform.parent = parent;
        Destroy(meshObject.GetComponent<MeshCollider>());
        SetVisible(false);
      }

      public void UpdateTerrainChunk(){
        float viewerDstFromNearEdge = Mathf.Sqrt(bounds.SqrDistance(viewerPosition));
        bool visible = viewerDstFromNearEdge<=maxViewDst;
        SetVisible(visible);
      }

      public void SetVisible(bool visible){
        meshObject.SetActive(visible);
      }

      public bool isVisible(){
        return meshObject.activeSelf;
      }


    }

}
