using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LCM;

public class quaruped_paramter_publish: MonoBehaviour {

	int t = 0;
	LCM.LCM.LCM myLCM;

	// Use this for initialization
	void Start () {
		Debug.Log("Publish start");
		myLCM = LCM.LCM.LCM.Singleton;
	}
	
	// Update is called once per frame
	void Update () {
		t++;
		if (t > 0){
			LCMTypes.quadruped_parameters_lcmt msg = new LCMTypes.quadruped_parameters_lcmt();
			TimeSpan span = DateTime.Now - new DateTime(1970, 1, 1);
      if(Input.GetKeyDown(KeyCode.Keypad1)){
        msg.control_mode = 1;
      }else if(Input.GetKeyDown(KeyCode.Keypad2)){
        msg.control_mode = 2;
      }else if(Input.GetKeyDown(KeyCode.Keypad3)){
        msg.control_mode = 3;
      }else if(Input.GetKeyDown(KeyCode.Keypad4)){
        msg.control_mode = 4;
      }else if(Input.GetKeyDown(KeyCode.Keypad5)){
        msg.control_mode = 5;
      }else if(Input.GetKeyDown(KeyCode.Keypad6)){
        msg.control_mode = 6;
      }else if(Input.GetKeyDown(KeyCode.Keypad7)){
        msg.control_mode = 7;
      }else if(Input.GetKeyDown(KeyCode.Keypad8)){
        msg.control_mode = 8;
      }

      if(Input.GetKeyDown(KeyCode.UpArrow)){
        msg.key_vertical = 1.0f;
      }





			
			myLCM.Publish("quadruped_parameters", msg);
			t=0;
		}
	}
}
