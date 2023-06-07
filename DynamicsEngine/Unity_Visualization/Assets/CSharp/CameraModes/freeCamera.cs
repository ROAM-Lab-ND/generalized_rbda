using UnityEngine;
using System.Collections;

// To Fix: not completely free camera, still moves with robot body
// Bug or a feature?

public class freeCamera : MonoBehaviour
{
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
    public float PanSensitivity = 3f;
    private bool CameraDisabled = false;

    // Use this for initialization
    void Start()
    {
        this._XForm_Camera = this.transform;
        this._XForm_Parent = this.transform.parent;
        this._LocalRotation[1] = 15.0f; // angle offset
    }

    void LateUpdate()
    {
        if(this.gameObject.GetComponent<Camera>().enabled && !MenuData.instance.gameObject.activeInHierarchy){
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
                    _LocalRotation.y = Mathf.Clamp(_LocalRotation.y,0f,120f);
                }

            }

    //Zooming Input from our Mouse Scroll Wheel
           if (Input.GetAxis("Mouse ScrollWheel") != 0f)
                {
                    float ScrollAmount = Input.GetAxis("Mouse ScrollWheel") * ScrollSensitvity;
                    ScrollAmount *= (this._CameraDistance * 0.3f);
                    this._CameraDistance += ScrollAmount * -1f;
                    this._CameraDistance = Mathf.Clamp(this._CameraDistance, 1.5f, 100f);
                }

            //Actual Camera Rig Transformations
            Quaternion QT = Quaternion.Euler(_LocalRotation.y, _LocalRotation.x, 0);
            this._XForm_Parent.rotation = Quaternion.Lerp(this._XForm_Parent.rotation, QT, Time.deltaTime * OrbitDampening);

            // camera panning
            if(Input.GetKey(KeyCode.W)){
              this._XForm_Camera.localPosition += Time.deltaTime*Vector3.forward*PanSensitivity;
            }
            if(Input.GetKey(KeyCode.S)){
              this._XForm_Camera.localPosition += Time.deltaTime*Vector3.back*PanSensitivity;
            }
            if(Input.GetKey(KeyCode.A)){
              this._XForm_Camera.localPosition += Time.deltaTime*Vector3.left*PanSensitivity;
            }
            if(Input.GetKey(KeyCode.D)){
              this._XForm_Camera.localPosition += Time.deltaTime*Vector3.right*PanSensitivity;
            }
        }
    }
}
