using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Page : MonoBehaviour
{
  public void toggleActive(){
    gameObject.SetActive(!gameObject.activeInHierarchy);
  }
}
