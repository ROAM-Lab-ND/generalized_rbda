using System.Collections;
using System.Collections.Generic;
//using System.Collections.IEnumerable;
using UnityEngine;
using System;
using LCM;

public class debugObjectVisualization : MonoBehaviour
{
    /* Push Disturbance Arrows */
    GameObject DisturbanceVector;
    static int disturbance_count;
    private static Vector3 body_position     = new Vector3 (0f,0f,0f);
    private static Vector3 disturbance_force = new Vector3 (0f,0f,0f);
    private static Vector3 disturbance_scale = new Vector3 (0f,0f,0f);
    private static Color disturbance_color = new Color (0f,1f,0f,0.5f);
//    private float scaleFactorDisturbance = 500;  

    /* Arrow Variables & Parameters */
    private static int arrow_count;
    private static int last_arrow_count;
    private static int max_arrows=100;
    Vector3 default_arrow_scale = new Vector3(0.1f,0.1f,0.1f);
    Vector3 default_arrow_position = new Vector3(0,-2,0);
    Color default_arrow_color = new Color(1.0f,0.0f,0.0f,1.0f);
    private GameObject arrow;
    public List<GameObject> arrows = new List<GameObject>();
    private static Renderer[] arrow_renderers= new Renderer[max_arrows]; //caching the renderers so it doesn't have to getComponent on update (not sure if necessary)
    private static Vector3[] arrow_positions= new Vector3[max_arrows];
    private static Vector3[] arrow_directions= new Vector3[max_arrows];
    private static Vector3[] arrow_scales= new Vector3[max_arrows];
    private static Color[] arrow_colors= new Color[max_arrows];
    private static float[] arrow_nominalMagnitudes = new float[max_arrows];

    /* Sphere Variables & Parameters */
    private static int sphere_count;
    private static int last_sphere_count;
    private static int max_spheres=100;
    Vector3 default_sphere_scale = new Vector3(0.1f,0.1f,0.1f);
    Vector3 default_sphere_position = new Vector3(0,-2,0);
    Color default_sphere_color = new Color(0.0f,0.0f,1.0f,.50f);
    private GameObject sphere;
    public List<GameObject> spheres = new List<GameObject>();
    private static Renderer[] sphere_renderers= new Renderer[max_spheres]; //caching the renderers so it doesn't have to getComponent on update (not sure if necessary)
    private static Vector3[] sphere_positions= new Vector3[max_spheres];
    private static Vector3[] sphere_scales= new Vector3[max_spheres];
    private static Color[] sphere_colors= new Color[max_spheres];

    /* Obstacle Variables & Parameters */
    private static int obst_count;
    private static int last_obst_count;
    private static int max_obst=4;
    Vector3 default_obst_scale = new Vector3(0.1f,0.1f,0.1f);
    Vector3 default_obst_position = new Vector3(0f,-2.5f,0f);
    Vector3 default_obst_rpy = new Vector3(0f,0f,0f);
    Color default_obst_color = new Color(0.0f,1.0f,1.0f,.50f);
    private GameObject obstacle;
    public List<GameObject> obstacles = new List<GameObject>();
    private static Renderer[] obst_renderers= new Renderer[max_obst]; //caching the renderers so it doesn't have to getComponent on update (not sure if necessary)
    private static Vector3[] obst_positions= new Vector3[max_obst];
    private static Vector3[] obst_scales= new Vector3[max_obst];
    private static Vector3[] obst_oris= new Vector3[max_obst];
    private static Color[] obst_colors= new Color[max_obst];

    /* Path Variables & Parameters */
    private static int path_count;
    private static int last_path_count;
    private static int max_paths=50;
    private static int max_path_points=200;
    Color default_path_color = new Color(1.0f,1.0f,0.0f,.50f);
    private GameObject path;
    private LineRenderer line;
    private LineRenderer[] lines=new LineRenderer[max_paths]; //caching the renderers so it doesn't have to getComponent on update
    public List<GameObject> paths = new List<GameObject>();
    private static Vector3[,] path_positions= new Vector3[max_paths,max_path_points];
    private static int[] path_lengths= new int[max_paths];
    private static float[] path_widths= new float[max_paths];
    private static Color[] path_colors= new Color[max_paths];

    // Start is called before the first frame update
    void Start()
    {
      StartCoroutine("Listener");
      StartSpheres();
      StartObstacles();
      StartPaths();
      StartArrows();
    }

    // Update is called once per frame
    void Update()
    {
      UpdateSpheres();
      UpdateObstacles();
      UpdatePaths();
      UpdateArrows();

    }

    private void StartSpheres(){ //Initialize sphere game objects
      for(int sphere_idx = 0; sphere_idx<max_spheres; ++sphere_idx){
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = default_sphere_position;
        sphere.transform.localScale = default_sphere_scale;
        sphere_renderers[sphere_idx]=sphere.GetComponent<Renderer>();
        sphere_renderers[sphere_idx].material=new Material(Shader.Find("Sprites/Default"));
        sphere_renderers[sphere_idx].material.color=default_sphere_color;
        Destroy(sphere.GetComponent<SphereCollider>());
        sphere.transform.SetParent(gameObject.transform.Find("Spheres"));
        spheres.Add(sphere);
      }
    }

    private void StartObstacles(){ //Initialize sphere game objects
      for(int obst_idx = 0; obst_idx<max_obst; ++obst_idx){
        obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.transform.position = default_obst_position;
        obstacle.transform.localScale = default_obst_scale;
        obstacle.transform.localEulerAngles = default_obst_rpy;
        obst_renderers[obst_idx]=obstacle.GetComponent<Renderer>();
        obst_renderers[obst_idx].material=new Material(Shader.Find("Sprites/Default"));
        obst_renderers[obst_idx].material.color=default_obst_color;
        Destroy(obstacle.GetComponent<BoxCollider>());
        obstacle.transform.SetParent(gameObject.transform.Find("Obstacles"));
        obstacles.Add(obstacle);
      }
    }

    private void StartPaths(){ //Initialize path game objects
      for(int path_idx = 0; path_idx<max_paths; ++path_idx){
        path = new GameObject("path"); 
        lines[path_idx] = path.AddComponent<LineRenderer>();
        lines[path_idx].widthMultiplier = 0.01f;
        lines[path_idx].positionCount = 0;
        //Set the paths default material/color
        lines[path_idx].material = new Material(Shader.Find("Sprites/Default"));
        lines[path_idx].material.color = default_path_color;
        path.transform.SetParent(gameObject.transform.Find("Paths"));
        paths.Add(path);
      }
    }

    private void StartArrows(){
      GameObject arrowTemplate = (GameObject.Find("arrowTemplate"));
      for(int arrow_idx = 0; arrow_idx<max_arrows; ++arrow_idx){
	arrow = Instantiate(arrowTemplate, gameObject.transform.Find("Arrows"));
	arrow.name = "arrow";
        arrow.transform.position = default_arrow_position;
        arrow.transform.localScale = default_arrow_scale;
	foreach(Renderer r in arrow.GetComponentsInChildren<Renderer>()){
	    r.material.color= default_arrow_color;
	}
        arrows.Add(arrow);
      }
    }

    private void UpdateSpheres(){
      //Clear spheres -> move spheres from previous frame to default location (beneath the map)
      for(int sphere_idx = 0; sphere_idx<last_sphere_count; ++sphere_idx){
        spheres[sphere_idx].transform.position = default_sphere_position;
        spheres[sphere_idx].transform.localScale = default_sphere_scale;
      }
      // Update spheres with LCM information
      if(sphere_count>0){
        for(int sphere_idx = 0; sphere_idx<sphere_count; ++sphere_idx){
          spheres[sphere_idx].transform.position = sphere_positions[sphere_idx]; 
          spheres[sphere_idx].transform.localScale = sphere_scales[sphere_idx];
          sphere_renderers[sphere_idx].material.color=sphere_colors[sphere_idx];
        }
      }
      last_sphere_count=sphere_count;

    }

    private void UpdateObstacles(){
      //Clear obstacles -> move obstacles from previous frame to default location (beneath the map)
      for(int obst_idx = 0; obst_idx<last_obst_count; ++obst_idx){
        spheres[obst_idx].transform.position = default_obst_position;
        spheres[obst_idx].transform.localScale = default_obst_scale;
      }
      // Update spheres with LCM information
      if(obst_count>0){
        for(int obst_idx = 0; obst_idx<obst_count; ++obst_idx){
          obstacles[obst_idx].transform.position = obst_positions[obst_idx];
          obstacles[obst_idx].transform.localScale = obst_scales[obst_idx];
          obstacles[obst_idx].transform.localEulerAngles = obst_oris[obst_idx];
          obst_renderers[obst_idx].material.color= obst_colors[obst_idx];
        }
      }
      last_obst_count=obst_count;

    }

    private void UpdatePaths(){
      // Clear Paths -> Reset the path count from paths in previous frames
      for(int path_idx = 0; path_idx<last_path_count; ++path_idx){
        lines[path_idx].positionCount = 0;
      }
      // Update Paths with LCM information
      if(path_count>0){
        for(int path_idx = 0; path_idx<path_count; ++path_idx){
          lines[path_idx].positionCount = path_lengths[path_idx];
          Vector3[] current_path_positions= new Vector3[path_lengths[path_idx]]; //create array of current path_positions(inefficient but good for now)
          for(int point_idx=0;point_idx<path_lengths[path_idx]; ++point_idx){
            current_path_positions[point_idx]=path_positions[path_idx,point_idx];
          }
          lines[path_idx].SetPositions(current_path_positions); //assign positions
          //change color
          lines[path_idx].material.color=path_colors[path_idx];
          lines[path_idx].widthMultiplier=path_widths[path_idx];
        }
      }
      last_path_count=path_count;
    }

    private void UpdateArrows(){
      //Clear arrows -> move arrows from previous frame to default location (beneath the map)
      for(int arrow_idx = 0; arrow_idx<last_arrow_count; ++arrow_idx){
        arrows[arrow_idx].transform.position = default_arrow_position;
        arrows[arrow_idx].transform.localScale = default_arrow_scale;
      }
      // Update arrows with LCM information
      if(arrow_count>0){
        for(int arrow_idx = 0; arrow_idx<arrow_count; ++arrow_idx){
	  float arrow_norm = arrow_directions[arrow_idx].magnitude;
	  float arrow_ratio = arrow_norm/(Mathf.Pow(arrow_nominalMagnitudes[arrow_idx], 2.0f));
	  if(arrow_ratio > 0.05f){
            arrows[arrow_idx].transform.position = arrow_positions[arrow_idx];
	    arrows[arrow_idx].transform.rotation = Quaternion.FromToRotation(new Vector3(0.0f, 1.0f, 0.0f), arrow_directions[arrow_idx]);
	    // Mischief: hard code GRF arrow scaling
      float scale = 0.004f * arrow_norm;
      //arrow_norm/(10*Mathf.Pow(arrow_nominalMagnitudes[arrow_idx],2.0f));
  	    arrows[arrow_idx].transform.localScale = new Vector3(0.1f, scale, 0.1f);
            // do we need to update the colors of the arrows live? is there a good use case? can we just instantiate an arrow with a desired color at the start?
	    foreach(Renderer r in arrows[arrow_idx].GetComponentsInChildren<Renderer>()){
	        r.material.color= arrow_colors[arrow_idx];
	    }
	  } else {
	    arrows[arrow_idx].transform.localScale = Vector3.zero;
          }
        }
      }
      last_arrow_count=arrow_count;
    }

    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

      public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
      {
        // Debug.Log ("RECV: " + channel);
        if (channel == "debug_visualization"){
          LCMTypes.debug_visualization_lcmt msg = new LCMTypes.debug_visualization_lcmt(dins);
          // Arrows
          arrow_count = msg.arrow_count;
          if(msg.arrow_count > 0){
            for(int arrow_idx = 0; arrow_idx < arrow_count; ++arrow_idx){
              arrow_positions[arrow_idx] = new Vector3(msg.arrow_base_positions[arrow_idx*3],
                        msg.arrow_base_positions[arrow_idx*3+2],
                        msg.arrow_base_positions[arrow_idx*3+1]);
              arrow_directions[arrow_idx] = new Vector3(msg.arrow_directions[arrow_idx*3],
                        msg.arrow_directions[arrow_idx*3+2],
                        msg.arrow_directions[arrow_idx*3+1]);
              arrow_colors[arrow_idx] = new Color(msg.arrow_colors[arrow_idx*4],
                                                        msg.arrow_colors[arrow_idx*4+1],
                                                        msg.arrow_colors[arrow_idx*4+2],
                                                        msg.arrow_colors[arrow_idx*4+3]);
              arrow_nominalMagnitudes[arrow_idx] = msg.arrow_nom[arrow_idx];
            }
          }
          
          // Spheres
          sphere_count=msg.sphere_count;
          if(msg.sphere_count>0){
            for(int sphere_idx = 0; sphere_idx<sphere_count; ++sphere_idx){
              sphere_positions[sphere_idx]=new Vector3(msg.sphere_positions[sphere_idx*3],
                                                        msg.sphere_positions[sphere_idx*3+2],
                                                        msg.sphere_positions[sphere_idx*3+1]);
              sphere_scales[sphere_idx]=new Vector3(msg.sphere_radii[sphere_idx],
                                                    msg.sphere_radii[sphere_idx],
                                                    msg.sphere_radii[sphere_idx]);
              sphere_colors[sphere_idx]=new Color(msg.sphere_colors[sphere_idx*4],
                                                    msg.sphere_colors[sphere_idx*4+1],
                                                    msg.sphere_colors[sphere_idx*4+2],
                                                    msg.sphere_colors[sphere_idx*4+3]);
            }
          }

          // Obstacles
          obst_count=msg.obst_count;
          if(msg.obst_count>0){
            for(int obst_idx = 0; obst_idx<obst_count; ++obst_idx){
              obst_positions[obst_idx]=new Vector3(msg.obst_positions[obst_idx*3],
                                                        msg.obst_positions[obst_idx*3+2],
                                                        msg.obst_positions[obst_idx*3+1]);
              obst_oris[obst_idx]=new Vector3(57.296f*msg.obst_oris[obst_idx*3],
                                                        -57.296f*msg.obst_oris[obst_idx*3+2],
                                                        57.296f*msg.obst_oris[obst_idx*3+1]); // rad2deg
              obst_scales[obst_idx]=new Vector3(msg.obst_dims[obst_idx*3],
                                                    msg.obst_dims[obst_idx*3+2],
                                                    msg.obst_dims[obst_idx*3+1]);
              obst_colors[obst_idx]=new Color(msg.obst_colors[obst_idx*4],
                                                    msg.obst_colors[obst_idx*4+1],
                                                    msg.obst_colors[obst_idx*4+2],
                                                    msg.obst_colors[obst_idx*4+3]);
            }
          }
          
          // Paths
          path_count=msg.path_count;
          if(msg.path_count>0){
            path_lengths=msg.path_lengths;
            path_widths=msg.path_widths;
            for(int path_idx = 0; path_idx<path_count; ++path_idx){
              //colors and sizes
              path_colors[path_idx]=new Color(msg.path_colors[path_idx*4],
                                              msg.path_colors[path_idx*4+1],
                                              msg.path_colors[path_idx*4+2],
                                              msg.path_colors[path_idx*4+3]);
              for(int point_idx=0;point_idx<msg.path_lengths[path_idx]; ++point_idx){
                path_positions[path_idx,point_idx]=new Vector3(msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3],
                                                                msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3+2],
                                                                msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3+1]);
              }
            }
          }
        }
      }
    }




    Boolean running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener()
    {
      Debug.Log ("Debug listener started.");
      myLCM = new LCM.LCM.LCM();

      myLCM.Subscribe("debug_visualization", new SimpleSubscriber());
      // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
      running = true;
      while (running){
        yield return null;
      }
      Debug.Log ("Debug listener coroutine returning!");
    }
  }
