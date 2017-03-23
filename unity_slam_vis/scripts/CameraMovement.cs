using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;

public class CameraMovement : MonoBehaviour {

	private float panSpeed = 1.0f;
	private float zoomSpeed = 1.0f;
	private float rotSpeed = 1.0f;

	float sens = 10.0f;
	RaycastHit hit;
	bool isHit=false;
	Camera mCamera;

	void Start(){
		transform.position = new Vector3 (0, 5, -10f);
		transform.LookAt (new Vector3 (0, 0, 0));
		mCamera = gameObject.GetComponents<Camera> ()[0];
	}

	void OnPreRender() {
		//GL.wireframe = true;
	}
	void OnPostRender() {
		GL.wireframe = false;
	}

	public void CloseWatch(Transform tar){
		CloseWatch (tar.position);
	}

	public void CloseWatch(Vector3 pos){
		transform.position = new Vector3 (pos.x, pos.y+10, pos.z-10);
		transform.LookAt (pos);
	}

	private bool IsOnUI() {
		return EventSystem.current.IsPointerOverGameObject();
	}

	void Update () {
		if (IsOnUI ()) {
			return;
		}
		float scroll = Input.GetAxis("Mouse ScrollWheel");
		if (scroll != 0) {			
			scroll = scroll * zoomSpeed * sens;
			moveForward (scroll);
		}

		if (Input.GetMouseButton (2)) {
			float panD_X = -Input.GetAxis ("Mouse X") * sens *panSpeed;
			float panD_Y = -Input.GetAxis ("Mouse Y") * sens *panSpeed;
			panX (panD_X);
			panY (panD_Y);
		}

		if (Input.GetMouseButton (0)) {
			float panD_X = -Input.GetAxis ("Mouse X") * sens *rotSpeed;
			float panD_Y = -Input.GetAxis ("Mouse Y") * sens *rotSpeed;
			transform.eulerAngles += new Vector3 (panD_Y, -panD_X, 0);
		}

		if (Input.GetMouseButtonDown (1)) {
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			isHit = Physics.Raycast (ray,out hit);
		}

		if (Input.GetMouseButton (1)) {
			if (isHit) {
				float panD_X = Input.GetAxis ("Mouse X") * sens *rotSpeed;
				float panD_Y = Input.GetAxis ("Mouse Y") * sens *rotSpeed;
				transform.RotateAround (hit.point,new Vector3(0.0f,1.0f,0.0f),panD_X);
				transform.RotateAround (hit.point,transform.right,-panD_Y);
				transform.LookAt (hit.point);
			}
		}

		if (Input.GetMouseButtonUp (1)) {
			isHit = false;
		}

		if (Input.GetKey (KeyCode.W)) {
			float amount = Time.deltaTime * zoomSpeed * sens*2;
			moveForward (amount);
		}

		if (Input.GetKey (KeyCode.S)) {
			float amount = Time.deltaTime * zoomSpeed * sens*2;
			moveForward (-amount);
		}

		if (Input.GetKey (KeyCode.A)) {
			float amount = -Time.deltaTime * panSpeed * sens*2;
			panX (amount);
		}

		if (Input.GetKey (KeyCode.D)) {
			float amount = Time.deltaTime * panSpeed * sens*2;
			panX (amount);
		}

		if (Input.GetKey (KeyCode.Q)) {
			float amount = Time.deltaTime * panSpeed * sens*2;
			panY (amount);
		}

		if (Input.GetKey (KeyCode.E)) {
			float amount = -Time.deltaTime * panSpeed * sens*2;
			panY (amount);
		}

		if (Input.GetMouseButtonDown (0)) {
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			if (Physics.Raycast (ray, out hit)) {
				if (hit.collider.gameObject) {
					camModel camObj = hit.collider.gameObject.GetComponent<camModel> ();
					if (camObj != null) {
						ChannelInfo item =ChannelInfo.getChannel (camObj.listId);
						item.ShowKFInfo (camObj.keyFrameId);
						item.ShowKFConnect (camObj.keyFrameId);
						item.ShowMPConnect (camObj.keyFrameId);
					}
				}
			}
		}
	}
	void panX(float amount){
		transform.position += transform.right.normalized * amount;
	}

	void panY(float amount){
		transform.position += transform.up.normalized * amount;
	}

	void moveForward(float amount){
		transform.Translate(Vector3.forward * amount);
	}
}
