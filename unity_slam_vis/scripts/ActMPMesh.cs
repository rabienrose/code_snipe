using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ActMPMesh : MonoBehaviour {

	public static Transform rootObj;
	public static Transform create(List<ActMapPoint> info){
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "AMPRoot";
		}
		int itemCount = info.Count;
		for (int i = 0; i < itemCount; i++) {
			GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obj.transform.SetParent (rootObj);
			obj.name = "AMPMesh";
			obj.transform.position = info [i].pos;
			obj.transform.localScale = new Vector3(0.5f,0.5f,0.5f);
			obj.GetComponent<MeshRenderer> ().material = commonObj.mActMPM;
			//obj.GetComponent<MeshRenderer> ().material.color = info [i].color;
			obj.AddComponent<ActMPMesh>();

		}
		return rootObj;
	}
	void Start () {
	
	}

	public void ChangeColor(Color32 color, int minValue, int maxValue, int type){

	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
