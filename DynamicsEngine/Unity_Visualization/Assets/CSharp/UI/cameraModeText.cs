using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class cameraModeText : MonoBehaviour
{
    public GameObject Cameras; // cameras parent gameObject
    public Canvas UI; // ui gameObject
    public Text displayText; // camera mode display text

    private const int numStates = 3; //number of camera modes
    private Camera[] modes = new Camera[numStates]; // camera modes
    private int currentMode; // current camera mode

    void Start()
    {
      // stores all camera modes in array
      for(int i=0;i<numStates;i++){
        this.modes[i] = Cameras.transform.GetChild(i).gameObject.GetComponent<Camera>();
      }
    }

    void Update()
    {
      currentMode = Cameras.GetComponent<CameraSelection>().cameraState;
      switch(currentMode){
        case 0:
          UI.worldCamera = modes[0];
          displayText.text = "Camera Mode: Tracking";
          break;
        case 1:
          UI.worldCamera = modes[1];
          displayText.text = "Camera Mode: Free";
          break;
        case 2:
          UI.worldCamera = modes[2];
          displayText.text = "Camera Mode: First person";
          break;
      }
    }
}
