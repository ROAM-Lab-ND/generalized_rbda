using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;
//using csharp;
//using c_test_lcm;

public class joint_update_test: MonoBehaviour
{
  public static int num_joint = 13;
  public static int count = 0;
  public Transform[] joint_trans = new Transform[num_joint];
  public static float[,] joint_ori_quat = new float[num_joint,4];
  public static float[,] joint_pos = new float[num_joint,3];

  // Start is called before the first frame update
  void Start()
  {
    StartCoroutine("Listener");
    //for(int i=0; i<num_joint; ++i){
    //for(int i=0; i<6; ++i){
      //this.joint_trans[i] = this.transform.GetChild(1); //body
    //}
    this.joint_trans[0] = this.transform.GetChild(1); 
    for(int i=1; i<7; ++i){
      this.joint_trans[i] = this.joint_trans[i-1].GetChild(1);
    }
    this.joint_trans[7] = this.joint_trans[5].GetChild(2);
  }

  // Update is called once per frame
  void Update()
  {
    count = count +1;
    for(int joint_idx = 0; joint_idx<num_joint; ++joint_idx){
      //this.joint_trans[joint_idx].localPosition =
        //new Vector3(joint_pos[joint_idx, 0], joint_pos[joint_idx, 2], joint_pos[joint_idx, 1]);

      Vector3 angle = new Vector3(0f, 0f, 0f);
      angle[1] = 40*(float)Math.Sin(count*0.002);
      this.joint_trans[0].localRotation = Quaternion.Euler(angle);

      angle[0] = 80*(float)Math.Sin(count*0.001);
      angle[1] = 0;
      this.joint_trans[1].localRotation = Quaternion.Euler(angle);

      angle[0] = 0; 
      angle[1] = 90*(float)Math.Sin(count*0.003);
      this.joint_trans[2].localRotation = Quaternion.Euler(angle);

      angle[0] = -80*(float)Math.Sin(count*0.001);
      angle[1] = 0;
      this.joint_trans[3].localRotation = Quaternion.Euler(angle);

      angle[0] = -80*(float)Math.Sin(count*0.001);
      angle[1] = 0; 
      this.joint_trans[4].localRotation = Quaternion.Euler(angle);

      // Wrist Roll
      angle[0] = 0; 
      angle[1] = -80*(float)Math.Sin(count*0.001);
      this.joint_trans[5].localRotation = Quaternion.Euler(angle);

      angle[1] = 0; 
      angle[2] = -25*(float)Math.Sin(count*0.004);
      this.joint_trans[7].localRotation = Quaternion.Euler(angle);
     }
  }

  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {

    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      //Debug.Log ("RECV: " + channel);
      //if (channel == "quadruped_visualization_info")
      //{
        //LCMTypes.quadruped_visualization_info_lcmt msg =
          //new LCMTypes.quadruped_visualization_info_lcmt(dins);

        //for(int joint_id = 0; joint_id<num_joint; ++joint_id){
          //for(int j=0; j<3; ++j){
            //joint_pos[joint_id, j] = msg.joint_pos[joint_id, j];
            //joint_ori_quat[joint_id, j] = msg.joint_ori_quat[joint_id, j];
          //}
          //joint_ori_quat[joint_id, 3] = msg.joint_ori_quat[joint_id, 3];
        //}
      //}

    }
  }
  Boolean running = false;
  LCM.LCM.LCM myLCM = null;

  public IEnumerator Listener()
  {
    Debug.Log ("Quadruped listener started.");
    myLCM = new LCM.LCM.LCM();

    myLCM.SubscribeAll(new SimpleSubscriber());
    // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
    running = true;
    while (running){
      yield return null;
    }
    Debug.Log ("Quadruped listener coroutine returning!");
  }

}
