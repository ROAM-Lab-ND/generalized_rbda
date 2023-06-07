using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* TO DO:
Add additional camera states that could be added
Put all camera states into array
Abstract away number of camera states into state variable
*/

public class CameraSelection : MonoBehaviour
{
    // posn and rotation of tracking camera before changing modes
    private Vector3 lastTCPosn;
    private Quaternion lastTCRot;

    private const int numStates = 3; // number of camera modes
    public int cameraState = 0; // initialize camera mode

    public Camera[] modes = new Camera[numStates];

    // function to enable active camera and disable others
    public void activate(int cameraState){
      switch (cameraState){
        case 0:
          //Debug.Log("Tracking camera.");
          break;
        case 1:
          // seamless transition between tracking and free camera
          // hierarchy dependent and NOT abstracted away
          lastTCPosn = modes[0].transform.position;
          lastTCRot = modes[0].transform.rotation;
          modes[1].transform.position = lastTCPosn;
          modes[1].transform.rotation = lastTCRot;
          //Debug.Log("Free camera.");
          break;
        case 2:
          //Debug.Log("First person camera.");
          break;
        // add more cases if more camera modes added in future

      }
      for (int i=0;i<numStates;i++){
        if (i==cameraState){
          modes[i].enabled = true;
        }
        else{
          modes[i].enabled = false;
        }
      }

    }

    void Start()
    {
      // stores all camera modes in hierarchy
      for(int i=0;i<numStates;i++){
        this.modes[i] = this.transform.GetChild(i).gameObject.GetComponent<Camera>();
      }
      cameraState = 0;
      activate(cameraState); // initialize in tracking camera mode
    }

    // Update is called once per frame
    void Update(){
      // change camera mode by pressing "c", cycle through
      if(Input.GetKeyDown(KeyCode.C)){
        cameraState = ((cameraState>=numStates-1) ? 0:(cameraState+1));
        Debug.Log("Camera state is " + cameraState);
        activate(cameraState);
      }
      //Debug.Log(cameraState);
    }
}
