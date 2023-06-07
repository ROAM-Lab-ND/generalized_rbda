using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using LCM;

public class simulationParamText : MonoBehaviour
{
    private static float realTime;
    private static double simTime;
    private static double simTime_pre;
    public float delayTime; // delay time bewteen parameter updates
    public Text displayText; //  time display text
    private int avgFrameRate; // FPS counter
    public int frameRange = 60; // number of frames to avg over for fps
    public int count = 0;
    public static Boolean first_visit = true;
    public static string controls_text= "Press 'Esc' to see controls \n";

    int[] fpsBuffer;
	  int fpsBufferIndex;

    // Start is called before the first frame update
    void Start()
    {
      StartCoroutine("Listener");
      realTime = 0;
      simTime = 0;
      simTime_pre = -1;
      delayTime = 1f;
    }

    // Update is called once per frame
    void Update()
    {
      count += 1;

      if(count>10){
        simTime_pre = simTime;
        count = 0;
      }
      if (fpsBuffer == null || fpsBuffer.Length != frameRange) {
        InitializeBuffer();
      }
      UpdateBuffer();
      CalculateFPS();

      //if (simTime>0.004){
        //realTime += Time.deltaTime;
      //}
      //else if(simTime<0.004){
        //realTime = 0.004f;
      //}
      if(!first_visit){
        realTime += Time.deltaTime;
      }
      if(simTime == simTime_pre && count == 10){
        first_visit = true;
      }

      displayText.text = displayString(avgFrameRate,simTime,realTime);
    }


    // prevent string garbage buildup
    public string displayString(int fps, double simTime, float realTime){
      var strBuild = new System.Text.StringBuilder();
      strBuild.Append("Run Time: " + realTime.ToString("F2") + " s" + "\n"
      + "Sim Time:  " + simTime.ToString("F2") + " s" + "\n"
      + "FPS: " + fps.ToString() +"\n" + controls_text);
      return strBuild.ToString();
    }

    void UpdateBuffer () {
  		fpsBuffer[fpsBufferIndex++] = (int)(1f / Time.unscaledDeltaTime);
  		if (fpsBufferIndex >= frameRange) {
  			fpsBufferIndex = 0;
  		}
  	}

    void CalculateFPS () {
  		int sum = 0;
  		for (int i = 0; i < frameRange; i++) {
  			sum += fpsBuffer[i];
  		}
  		avgFrameRate = sum / frameRange;
  	}

    void InitializeBuffer () {
  		if (frameRange <= 0) {
  			frameRange = 1;
  		}
  		fpsBuffer = new int[frameRange];
  		fpsBufferIndex = 0;
  	}



    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

      public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
      {
        //Debug.Log ("RECV: " + channel);
        if (channel == "simulator_state")
        {
          LCMTypes.simulator_lcmt msg = new LCMTypes.simulator_lcmt(dins);
          simTime = msg.time;
          if(first_visit){
            realTime = (float)msg.time;
          }
          first_visit = false;
        }
      }
    }
    Boolean running = false;
    LCM.LCM.LCM myLCM = null;


    public IEnumerator Listener()
    {
      myLCM = new LCM.LCM.LCM();

      myLCM.SubscribeAll(new SimpleSubscriber());
      running = true;
      while (running){
        yield return null;
      }
    }
}
