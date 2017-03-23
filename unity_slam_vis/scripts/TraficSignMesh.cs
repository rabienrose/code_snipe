using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TraficSignMesh : MonoBehaviour {

	public static Transform rootObj;
	public static Transform create(List<TraficSign> info){
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "TSRoot";
		}
		for (int i = 0; i < info.Count; i++) {
			GameObject obj = GameObject.Instantiate(commonObj.signPrefab);
			obj.transform.SetParent (rootObj);
			obj.name = "TSMesh";
			TraficSignMesh self = obj.AddComponent<TraficSignMesh>();
			obj.transform.position = info [i].pos;
			obj.transform.forward = info [i].forward;
			Texture tex=  Resources.Load ("sign/"+info[i].typeId) as Texture;
			obj.transform.FindChild ("parking").GetComponent<MeshRenderer> ().material.mainTexture = tex;
		}
		return rootObj;
	}
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
