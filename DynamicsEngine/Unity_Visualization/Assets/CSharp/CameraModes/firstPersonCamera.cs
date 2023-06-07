using UnityEngine;
using System.Collections;

public class firstPersonCamera : MonoBehaviour
{
  public GameObject firstPerson;
  private Transform BodyTransform;
  private Vector3 cameraOffset;
  private Vector3 body_R_cam;
  private float SmoothFactor = 1f;

    // Use this for initialization
    void Start()
    {
      // camera position near "head" of mc, customize for different robots
      BodyTransform = firstPerson.GetComponent<Transform>();
      cameraOffset[0] = BodyTransform.position.x+0.2f;
      this.transform.position=BodyTransform.position;

      // rotation matrix in ZXY Euler Angles for camera orientation
      body_R_cam = new Vector3(0f,-90f,90f);
    }

    void LateUpdate()
    {
        if(this.gameObject.GetComponent<Camera>().enabled){
            // angle offset to look straight
            this.transform.rotation = BodyTransform.localRotation*Quaternion.Euler(body_R_cam);

            // stays with robot body
            Vector3 newPos = BodyTransform.position + cameraOffset;
            this.transform.position = Vector3.Slerp(transform.position,newPos,SmoothFactor);
        }
    }
}
