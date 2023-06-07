using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class coordFrame : MonoBehaviour
{
  void Update(){
    this.transform.rotation = Quaternion.LookRotation(Vector3.up,Vector3.forward);
      //transform.localRotation = Quaternion.Euler(-rotRef.transform.rotation.eulerAngles)*Quaternion.Euler(rotMat);
    }
}
