using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class manipulator_subscribe : MonoBehaviour
{
  public static int num_joint = 2;
  public Transform[] joint_trans = new Transform[num_joint];
  public static float[] jpos = new float[num_joint];

  // Start is called before the first frame update
  void Start()
  {
    StartCoroutine("Listener");
    this.joint_trans[0] = this.transform.GetChild(1);
    this.joint_trans[1] = this.transform.GetChild(1).GetChild(1);
  }

  // Update is called once per frame
  void Update()
  {
    Vector3 angle  = new Vector3(0f, 0f, 0f);
    for(int joint_idx = 0; joint_idx< num_joint; ++joint_idx){
      angle = Vector3.zero;
      angle[2] = jpos[joint_idx]*180f/(float)Math.PI;
      this.joint_trans[joint_idx].localRotation = Quaternion.Euler(angle);
    }
  }


  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {

    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      Debug.Log ("Recieved: " + channel);
      if (channel == "openchain_2dof")
      {
        LCMTypes.openchain2dof_lcmt msg = new LCMTypes.openchain2dof_lcmt(dins);
        String message = "Received message of the type openchain_2dof:\n ";
        message+=String.Format(" jpos = ({0:N}, {1:N})\n ", msg.q[0], msg.q[1]);
        Debug.Log (message);

        for(int i = 0; i<num_joint; ++i){  jpos[i] = msg.q[i]; }
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
