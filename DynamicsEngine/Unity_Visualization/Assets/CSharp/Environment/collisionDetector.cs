using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class collisionDetector : MonoBehaviour
{

    Color collision_color = new Color(1.0f,0.0f,0.0f,.50f);
    Material default_material;
    Rigidbody rb;
    MeshCollider coll;
    IDictionary <string, List<string>> ignoreDict;
    string obj_name;


    private void OnTriggerExit(Collider other)
    {
        this.gameObject.GetComponent<MeshRenderer>().material=default_material;
    }
    private void OnTriggerStay(Collider other)
    {
    if (! ignoreDict[obj_name].Contains(other.gameObject.name)){
        this.gameObject.GetComponent<MeshRenderer>().material = new Material(Shader.Find("Sprites/Default"));
        this.gameObject.GetComponent<MeshRenderer>().material.color =collision_color;
        }
    }
    private void OnTriggerEnter(Collider other)
    {
    if (! ignoreDict[obj_name].Contains(other.gameObject.name)){
        this.gameObject.GetComponent<MeshRenderer>().material = new Material(Shader.Find("Sprites/Default"));
        this.gameObject.GetComponent<MeshRenderer>().material.color =collision_color;
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        default_material =this.gameObject.GetComponent<MeshRenderer>().material;
        rb =this.gameObject.AddComponent<Rigidbody>();
        rb.useGravity=false;
        coll=this.gameObject.AddComponent<MeshCollider>();
        coll.convex=true;
        coll.isTrigger =true;
        obj_name=this.gameObject.name;
        ignoreDict=new Dictionary<string,List<string>>() {//manual for now bc it's easier. Can create directly from tree
        {"base", new List<string> {"right_hip_yaw","left_hip_yaw","right_shoulder","left_shoulder"} },

        {"right_hip_yaw", new List<string> {"base","right_hip_abad"} },
        {"right_hip_abad", new List<string> {"right_hip_yaw","right_upper_leg"} },
        {"right_upper_leg", new List<string> {"right_hip_abad","right_lower_leg"} },
        {"right_lower_leg", new List<string> {"right_upper_leg","right_foot"} },
        {"right_foot", new List<string> {"right_lower_leg"} },

        {"right_shoulder", new List<string> {"base","right_shoulder_2"} },
        {"right_shoulder_2", new List<string> {"right_shoulder","right_upper_arm"} },
        {"right_upper_arm", new List<string> {"right_shoulder_2","right_lower_arm"} },
        {"right_lower_arm", new List<string> {"right_upper_arm"} },

        {"left_hip_yaw", new List<string> {"base","left_hip_abad"} },
        {"left_hip_abad", new List<string> {"left_hip_yaw","left_upper_leg"} },
        {"left_upper_leg", new List<string> {"left_hip_abad","left_lower_leg"} },
        {"left_lower_leg", new List<string> {"left_upper_leg","left_foot"} },
        {"left_foot", new List<string> {"left_lower_leg"} },

        {"left_shoulder", new List<string> {"base","left_shoulder_2"} },
        {"left_shoulder_2", new List<string> {"left_shoulder","left_upper_arm"} },
        {"left_upper_arm", new List<string> {"left_shoulder_2","left_lower_arm"} },
        {"left_lower_arm", new List<string> {"left_upper_arm"} }
        };
    }

    // Update is called once per frame
    void Update()
    {

    }
}