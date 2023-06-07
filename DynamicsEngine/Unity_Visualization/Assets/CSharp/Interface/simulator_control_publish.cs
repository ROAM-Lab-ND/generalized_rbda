using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LCM;

public class simulator_control_publish : MonoBehaviour {

  LCM.LCM.LCM myLCM;
  LCMTypes.simulator_control_lcmt msg = new LCMTypes.simulator_control_lcmt();

  // Use this for initialization
  void Start () {
    myLCM = LCM.LCM.LCM.Singleton;

  }

  void OnGUI(){
    Event e = Event.current;
    if (e.isKey)
    {
      Debug.Log("Detected key code: " + e.keyCode);
    }
  }
  // Update is called once per frame
  void Update () {
    Boolean push_button = false;

    //sim speed up
    if(Input.GetKey(KeyCode.T)){
      if(!msg.key_sim_speed_up){
        Debug.Log("sim speed up");
        push_button = true;
        msg.key_sim_speed_up = true;
      }}else if(msg.key_sim_speed_up){
        Debug.Log("sim normalized");
        push_button = true;
        msg.key_sim_speed_up = false;
      }
    //sim speed down
    if(Input.GetKey(KeyCode.G)){
      if(!msg.key_sim_speed_down){
        Debug.Log("sim speed down");
        push_button = true;
        msg.key_sim_speed_down = true;
      }}else if(msg.key_sim_speed_down){
        Debug.Log("sim normalized");
        push_button = true;
        msg.key_sim_speed_down = false;
      }

    //pause simulator
    if(Input.GetKeyDown(KeyCode.Space)){
      Debug.Log("Pause/Resume");
      msg.key_pause = true;
      push_button = true;
    }else if (Input.GetKeyUp(KeyCode.Space)){
      push_button = true;
      msg.key_pause = false;
    }
    if(push_button){
      myLCM.Publish("simulator_control", msg);
    }
  }
}
