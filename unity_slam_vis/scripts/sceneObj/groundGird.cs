using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class groundGird : MonoBehaviour {

	public static Transform rootObj;
	public static Transform create(){
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "GroundGird";
		}
		int lineCount = 0;
		GameObject obj = new GameObject ();
		obj.AddComponent<groundGird> ();
		obj.transform.SetParent (rootObj);
		List<Vector3> verts = new List<Vector3> ();
		List<Color32> colors = new List<Color32> ();
		List<int> inds = new List<int> ();
		for (int j = -500; j < 500; j=j+20){
			lineCount++;
			Vector3 p1 = new Vector3 (-500,0,j);
			Vector3 p2 = new Vector3 (500,0,j);
			verts.Add (p1);
			verts.Add (p2);
			inds.Add ((lineCount-1) * 2);
			inds.Add ((lineCount-1) * 2 +1);
			colors.Add (new Color32 (0, 0, 0, 255));
			colors.Add (new Color32 (0, 0, 0, 255));
		}
		for (int j = -500; j < 500; j=j+20){
			lineCount++;
			Vector3 p1 = new Vector3 (j,0,-500);
			Vector3 p2 = new Vector3 (j,0,500);
			verts.Add (p1);
			verts.Add (p2);
			inds.Add ((lineCount-1) * 2);
			inds.Add ((lineCount-1) * 2 +1);
			colors.Add (new Color32 (0, 0, 0, 255));
			colors.Add (new Color32 (0, 0, 0, 255));
		}

		Mesh mesh = new Mesh ();
		mesh.SetVertices (verts);
		mesh.SetColors (colors);
		mesh.SetIndices (inds.ToArray (), MeshTopology.Lines, 0);
		obj.AddComponent<MeshFilter> ().mesh = mesh;
		obj.AddComponent<MeshRenderer> ().material.shader= commonObj.mShader;
		return obj.transform;

	}
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
