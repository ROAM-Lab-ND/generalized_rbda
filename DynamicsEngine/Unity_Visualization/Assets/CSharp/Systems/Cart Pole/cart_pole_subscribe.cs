using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class cart_pole_subscribe : MonoBehaviour
{
    public Transform link_first;
    public Transform link_second;
    //
    //protected Transform link_first;

    public static float[] link1_pos = new float[3];
    public static float[] link2_pos = new float[3];

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine("Listener");
        this.link_first = this.transform.GetChild(0);
        this.link_second = this.transform.GetChild(1);
        //Debug.Log(this.transform.childCount);
        //this.link2 = this.transform.GetChild(1);
    }

    // Update is called once per frame
    void Update()
    {
        //((Transform)link_first).position = new Vector3(0., 0.7, 0.);
        //this.link2.position = new Vector3(0., 0.4, 0.);
        //this.transform.GetChild(0).position = new Vector3(0,0.7, 0.);
        //Debug.Log("what");
        //this.transform.position = new Vector3(0,0.7, 0.);
        //float x = this.transform.position.x;
        //x = 3;
        //Vector3 c = this.transform.position;
        //this.transform.GetChild(0).position = new Vector3(link1_pos[0],link1_pos[1],0);
        this.link_first.localPosition = new Vector3(link1_pos[0],link1_pos[1],0);
        this.link_first.localRotation = Quaternion.Euler(0, 0, link1_pos[2]);

        this.link_second.localPosition = new Vector3(link2_pos[0],link2_pos[1],0);
        this.link_second.localRotation = Quaternion.Euler(0, 0, (float)(link2_pos[2]*180/Math.PI));
         //this.link_first.Rotate(3.0f, (float)0.3f, link1_pos[2], Space.Self);
        //this.transform.GetChild(0).Rotate(0., 0., link1_pos[2], Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.Rotate(0., 0., 0., Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.GetChild(0).rotation = Quaternion.Euler(0., 0., 0.); 
        //link_first.Rotate(0., 0., link1_pos[2], Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.GetChild(0).rotation = new Quaternion.Euler(0., link1_pos[2],0);
        //this.transform.GetChild(0).rotation = Quaternion.Euler(0., link1_pos[2], 0); // Identifier error
        //transform.GetChild(0).rotation = Quaternion.Euler(0., this.link1_pos[2], 0); // Identifier error
        //transform.GetChild(0).rotation = new Quaternion.Euler(0., 30, 0); // Identifier error
        //transform.GetChild(0).rotation = new Quaternion((float)0.3, 0, 0, (float)1.3); // Identifier error
        
        //this.link_first.rotation = new Quaternion(0, 0, 0, 1) // O.K.;
        //this.link_first.rotation = new Quaternion((float)Math.Sin(0.523), 0, 0, (float)Math.Cos(0.523)); // O.K.
        //this.link_first.rotation = Quaternion.Euler(0, 0, 30); // O.K.
        //transform.GetChild(0).rotation = new Quaternion(0, 0, 0, 1); // O.K.
    }


    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
        {
            Debug.Log ("RECV: " + channel);
            if (channel == "cart_pole")
            {
                LCMTypes.cart_pole_lcmt msg = new LCMTypes.cart_pole_lcmt(dins);
                String message = "Received message of the type cart_pole:\n ";
                message+=String.Format("  link 1 = ({0:N}, {1:N}, {2:N})\n ",
                        msg.link1_pos[0], msg.link1_pos[1], msg.link1_pos[2]);
                message+=String.Format("  link 2 = ({0:N}, {1:N}, {2:N})\n",
                        msg.link2_pos[0], msg.link2_pos[1], msg.link2_pos[2]);
                Debug.Log (message);

                for(int i = 0; i<3; ++i){
                    link1_pos[i] = msg.link1_pos[i];
                    link2_pos[i] = msg.link2_pos[i];
                }
            }
        }
    }
    bool running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener()
    {
        Debug.Log ("Listener started.");
        myLCM = new LCM.LCM.LCM();

        myLCM.SubscribeAll(new SimpleSubscriber());
        running = true;
        while (running){
            yield return null;
        }
        Debug.Log ("listener coroutine returning!");
    }

}
