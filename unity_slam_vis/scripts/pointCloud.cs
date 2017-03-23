using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class pointCloud : MonoBehaviour{
	static Shader mShader;
	public static Transform rootObj;
	public static void ClearAll(){
		if (rootObj != null) {
			GameObject.Destroy (rootObj.gameObject);
			rootObj = null;
		}
	}

	public static Transform create(List<mapPoint> pointlist){
		if (mShader == null) {
			mShader = Resources.Load ("pointShader") as Shader;
		}
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "CloudRoot";
		}
		GameObject obj = new GameObject ();
		obj.transform.SetParent (rootObj);
		obj.name = "pointCloud";
		pointCloud self = obj.AddComponent<pointCloud>();
		self.pointlist = pointlist;
		return obj.transform;
	}
	public List<mapPoint> pointlist;
	void Start () {
		bool morePoints = true;
		int curDealedCount = 0;
		int linePointsCount = pointlist.Count;

		while (morePoints) {
			List<Vector3> verts = new List<Vector3> ();
			List<Color32> colors = new List<Color32> ();
			List<int> inds = new List<int> ();
			if (linePointsCount - curDealedCount > 60000) {
				verts.Capacity = 60000;
				colors.Capacity = 60000;
				inds.Capacity = 60000;
			} else {
				morePoints = false;
			}
			int localCount =0;
			for (; curDealedCount < linePointsCount; curDealedCount++) {
				verts.Add (pointlist[curDealedCount].pos);
				colors.Add (pointlist[curDealedCount].color);
				inds.Add (localCount);
				localCount++;
				if (localCount >= 60000) {
					curDealedCount++;
					break;
				}
			}
			GameObject subMeshObj = new GameObject ();
			subMeshObj.name = "subMesh";
			Mesh mesh = new Mesh ();
			mesh.SetVertices (verts);
			mesh.SetColors (colors);
			mesh.SetIndices (inds.ToArray (), MeshTopology.Points, 0);
			subMeshObj.AddComponent<MeshFilter> ().mesh = mesh;
			subMeshObj.AddComponent<MeshRenderer> ().material.shader= mShader;
			subMeshObj.transform.SetParent (gameObject.transform);
		}
	
	}

	public void ChangeColor(Color32 color, int minValue, int maxValue, int type){
		int pointCount = 0;
		for (int n = 0; n < transform.childCount; n++) {
			Transform subObj = transform.GetChild (n);
			Mesh meshTemp = subObj.GetComponent<MeshFilter> ().mesh;
			Color32[] newColor = meshTemp.colors32;
			int maxCount = meshTemp.colors32.Length;
			for (int i = 0; i < maxCount; i++) {
				int val = 0;
				if (type == 0) {
					val = pointlist [pointCount].filter1;
				} else if (type == 1) {
					val = pointlist [pointCount].filter2;
				} else if (type == 2) {
					val = pointlist [pointCount].filter3;
				}
				if (maxValue > val && minValue < val) {
					newColor [i] = color;
				} else {
					//newColor [i] = meshTemp.colors32[i];
				}
				pointCount++;
			}
			meshTemp.colors32 = newColor;
		}
	}

	public void ChangeScale(int scale){
		for (int n = 0; n < transform.childCount; n++) {
			Transform subObj = transform.GetChild (n);
			Mesh meshTemp = subObj.GetComponent<MeshFilter> ().mesh;
			Vector3[] newVert = meshTemp.vertices;
			int tcount = meshTemp.colors32.Length;
			for (int i = 0; i < tcount; i++) {
				newVert [i] = newVert [i]*scale/100.0f;
			}
			meshTemp.vertices = newVert;
		}
		int count = pointlist.Count;
		for (int i = 0; i < count; i++) {
			mapPoint item = pointlist [i];
			item.pos = item.pos * scale / 100.0f;
			pointlist [i] = item;
		}
	}

	Bounds calBound(){
		float minX = 1000f, maxX = -1000f;
		float minY = 1000f, maxY = -1000f;
		float minZ = 1000f, maxZ = -1000f;
		for (int i = 0; i < pointlist.Count; i++) {
			if (pointlist [i].pos.x < minX) {
				minX = pointlist [i].pos.x;
			}
			if (pointlist [i].pos.x > maxX) {
				maxX = pointlist [i].pos.x;
			}
			if (pointlist [i].pos.y < minY) {
				minY = pointlist [i].pos.y;
			}
			if (pointlist [i].pos.y > maxY) {
				maxY = pointlist [i].pos.y;
			}
			if (pointlist [i].pos.z < minZ) {
				minZ = pointlist [i].pos.z;
			}
			if (pointlist [i].pos.z > maxZ) {
				maxZ = pointlist [i].pos.z;
			}
		}
		Bounds b = new Bounds ();
		b.SetMinMax (new Vector3 (minX, minY, minZ), new Vector3 (maxX, maxY, maxZ));
		return b;
	}



	public Vector3 GetCenterPos(){
		return calBound ().center;
	}

	void Update () {
	
	}
}
