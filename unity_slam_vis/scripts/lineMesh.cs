using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class lineMesh : MonoBehaviour {
	public static lineMesh create(){
		GameObject obj = new GameObject ();
		obj.name = "LineMesh";
		lineMesh item = obj.AddComponent<lineMesh> (); 
		item.color = new Color32(0,0,0, 255);
		return item;
	}

	List<Vector3> verts = new List<Vector3> ();
	List<Color32> colors = new List<Color32> ();
	List<int> inds = new List<int> ();
	int lineCount = 0;
	public Color32 color;

	public void createMesh(){
		if (verts.Count <= 0) {
			return;
		}
		Mesh mesh = new Mesh ();
		mesh.SetVertices (verts);
		mesh.SetColors (colors);
		mesh.SetIndices (inds.ToArray (), MeshTopology.Lines, 0);
		if (!gameObject.GetComponent<MeshFilter> ()) {
			gameObject.AddComponent<MeshFilter> ().mesh = mesh;
			gameObject.AddComponent<MeshRenderer> ().material.shader = commonObj.mShader;
		} else {
			gameObject.GetComponent<MeshFilter> ().mesh = mesh;
		}

	}

	public void addLine(Vector3 p1, Vector3 p2){
		lineCount = lineCount + 1;
		verts.Add (p1);
		verts.Add (p2);
		inds.Add ((lineCount-1) * 2);
		inds.Add ((lineCount-1) * 2 +1);
		colors.Add (color);
		colors.Add (color);
	}

	public void clearAllLine(){
		verts.Clear ();
		colors.Clear ();
		inds.Clear ();
		lineCount = 0;
	}

	public void ChangeColor(Color32 color){
		Mesh meshTemp = GetComponent<MeshFilter> ().mesh;
		Color32[] newColor = new Color32[meshTemp.colors32.Length];
		for (int i = 0; i < meshTemp.colors32.Length; i++) {
			newColor [i] = color;
		}
		meshTemp.colors32 = newColor;
	}

	public void changeScale(int scale){
		Mesh meshTemp = GetComponent<MeshFilter> ().mesh;
		Vector3[] newVert = meshTemp.vertices;
		for (int i = 0; i < newVert.Length; i++) {
			newVert [i] = newVert [i]*scale/100.0f;
		}
		meshTemp.vertices = newVert;
	}

	void Start () {
	
	}

	void Update () {
	
	}
}
