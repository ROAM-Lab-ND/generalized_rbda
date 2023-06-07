using UnityEngine;
using System.Collections;

public class trackingCamera : MonoBehaviour
{
    // in inspector, drag GameObject from left into "BodyTransform" attribute
    public Transform BodyTransform; // body that camera will follow, usually main body
    protected Transform _XForm_Camera; // actual POV
    protected Transform _XForm_Parent; // pivot point about which camera will rotate

    protected Vector3 _LocalRotation;
    protected float _CameraDistance = 2.0f; // vertical offset from pivot

    // Range allows custom values in inspector window before building

    [Range(0.01f,10.0f)]
    public float MouseSensitivity = 4f;

    [Range(0.01f,5.0f)]
    public float ScrollSensitvity = 2f;

    [Range(0.01f,20.0f)]
    public float OrbitDampening = 10f;

    [Range(0.01f,10.0f)]
    public float ScrollDampening = 6f;

    [Range(0.01f,1.0f)]
    public float SmoothFactor = 0.5f;

    private bool CameraDisabled = false;

    private Vector3 cameraOffset; // difference in camera and body position

    // Use this for initialization
    void Start()
    {
        this._XForm_Camera = this.transform;
        this._XForm_Parent = this.transform.parent;
        this._LocalRotation[1] = 15.0f; // planar offset of pivot from origin


        // offset distance to be maintained bw camera and body
        cameraOffset = this._XForm_Parent.position-BodyTransform.position;

    }

    void LateUpdate()
    {
        if(this.gameObject.GetComponent<Camera>().enabled){//&& !MenuData.instance.gameObject.activeInHierarchy){
            // Debug.Log("active");
            if (Input.GetMouseButton(0)) CameraDisabled = false;
    	      else CameraDisabled = true;

            if (!CameraDisabled)
            {
                //Rotation of the Camera based on Mouse Coordinates
                if (Input.GetAxis("Mouse X") != 0 || Input.GetAxis("Mouse Y") != 0)
                {
                    _LocalRotation.x += Input.GetAxis("Mouse X") * MouseSensitivity;
                    _LocalRotation.y += Input.GetAxis("Mouse Y") * MouseSensitivity;

                    //Clamp the y Rotation to horizon and not flipping over at the top
                    _LocalRotation.y = Mathf.Clamp(_LocalRotation.y,-20f,120f);
                }

            }

    //Zooming Input from our Mouse Scroll Wheel
           if (Input.GetAxis("Mouse ScrollWheel") != 0f)
                {
                    float ScrollAmount = Input.GetAxis("Mouse ScrollWheel") * ScrollSensitvity;
                    ScrollAmount *= (this._CameraDistance * 0.3f);
                    this._CameraDistance += ScrollAmount * -1f;
                    this._CameraDistance = Mathf.Clamp(this._CameraDistance, 0.5f, 100f);
                }

            //Actual Camera Rig Transformations
            Quaternion QT = Quaternion.Euler(_LocalRotation.y, _LocalRotation.x, 0);
            this._XForm_Parent.rotation = Quaternion.Lerp(this._XForm_Parent.rotation, QT, Time.deltaTime * OrbitDampening);

            Vector3 newPos = BodyTransform.position + cameraOffset;
            this._XForm_Parent.position = Vector3.Slerp(this._XForm_Parent.position,newPos,SmoothFactor);

            if (this._XForm_Camera.localPosition.z != this._CameraDistance * -1f)
            {
                this._XForm_Camera.localPosition = new Vector3(0.0f, 0.0f,
                Mathf.Lerp(this._XForm_Camera.localPosition.z,
                this._CameraDistance * -1f, Time.deltaTime * ScrollDampening));
            }
        }
    }
}
