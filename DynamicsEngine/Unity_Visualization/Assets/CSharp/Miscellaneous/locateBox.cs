using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class locateBox : MonoBehaviour
{
    public static Vector3 box_pos;
    public static Vector3 box_size;


    // Start is called before the first frame update
    void Start()
    {
	   StartCoroutine("Listener");
       box_pos[0] = 0f;
       box_pos[1] = -0.2f;
       box_pos[2] = 0f;
       box_size[0] = 0.1f;
       box_size[1] = 0.1f;
       box_size[2] = 0.1f;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = box_pos;
        transform.localScale = box_size;
    }

    // Subscriber Object
    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {
        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
        {
          if (channel == "simulator_objects")
          {
            LCMTypes.sim_box_info_lcmt msg = new LCMTypes.sim_box_info_lcmt(dins);

            box_pos[0] = msg.position[0];
            box_pos[1] = msg.position[2];
            box_pos[2] = msg.position[1];

            box_size[0] = msg.scale[0];
            box_size[1] = msg.scale[2];
            box_size[2] = msg.scale[1];

          }
        }
    }

    Boolean running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener()
    {
        Debug.Log ("Box info listener started.");
        myLCM = new LCM.LCM.LCM();
        myLCM.SubscribeAll(new SimpleSubscriber());
        // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
        running = true;
        while (running){
          yield return null;
        }
        Debug.Log ("Box Locater listener coroutine returning!");
    }
}
