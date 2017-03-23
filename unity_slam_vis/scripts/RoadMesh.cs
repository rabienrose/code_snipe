using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RoadMesh : MonoBehaviour {
	public static Transform rootObj;
	public Vector3 lookAtPoint;

	public List<Vector3> vertices;
	public List<Vector2> uv;
	public List<int> triang;
	public static RoadMesh create(List<RoadSeg> info){
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "RoadRoot";
		}
		GameObject obj = new GameObject ();
		obj.transform.SetParent (rootObj);
		obj.name = "RoadMesh";
		RoadMesh self = obj.AddComponent<RoadMesh>();

		self.vertices = new List<Vector3>();
		self.uv = new List<Vector2>();
		self.triang = new List<int>();
		obj.AddComponent<MeshFilter> ();
		obj.AddComponent<MeshRenderer> ().material = commonObj.mRoadSufM;
		self.addSegments(info);


		return self;
	}

	public void addSegments(List<RoadSeg> info){
		List<Vector3> roadP1List = new List<Vector3> ();
		List<Vector3> roadP2List = new List<Vector3> ();

		for (int i = 0; i < info.Count-1; i++) {
			Vector3 trajVec = info[i+1].pos - info [i].pos;
			Vector3 roadWidthVec = Vector3.Cross (trajVec, info [i].normal);
			roadWidthVec.Normalize ();
			Vector3 roadC = info[i].pos;
			Vector3 roadEdgeP1 = roadC + roadWidthVec * info[i].hWidthL;
			Vector3 roadEdgeP2 = roadC + -roadWidthVec * info[i].hWidthR;
			roadP1List.Add (roadEdgeP1);
			roadP2List.Add (roadEdgeP2);
			coorMark mark = coorMark.create (info [i].pos, roadWidthVec, info [i].normal,trajVec , i);
			mark.transform.SetParent (transform);
		}
//		vertices.Clear ();
//		uv.Clear();
//		triang.Clear();
		int startIndex = vertices.Count;
		vertices.Add (roadP1List [0]);
		uv.Add (new Vector2 (1f, 0f));
		vertices.Add (roadP2List [0]);
		uv.Add (new Vector2 (0f, 1f));

		for (int i = 0; i < info.Count-2; i++) {
			vertices.Add (roadP1List [i+1]);
			uv.Add (new Vector2 (1f, 0f));
			vertices.Add (roadP2List [i+1]);
			uv.Add (new Vector2 (0f, 1f));
			triang.Add (startIndex + i * 2+2);
			triang.Add (startIndex + i * 2+1);
			triang.Add (startIndex + i * 2+0);
			//double sides displaying
			triang.Add (startIndex + i * 2+0);
			triang.Add (startIndex + i * 2+1);
			triang.Add (startIndex + i * 2+2);

			triang.Add (startIndex + i * 2+1);
			triang.Add (startIndex + i * 2+2);
			triang.Add (startIndex + i * 2+3);
			//double sides displaying
			triang.Add (startIndex + i * 2+3);
			triang.Add (startIndex + i * 2+2);
			triang.Add (startIndex + i * 2+1);
		}

		Vector3[] vertBuf = vertices.ToArray ();
		int[] triangBuf = triang.ToArray();
		Vector2[] uvBuf = uv.ToArray ();
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		//Mesh mesh = new Mesh();
		mesh.vertices = vertBuf;
		mesh.triangles = triangBuf;
		mesh.uv = uvBuf;
		//GetComponent<MeshFilter> ().sharedMesh = mesh;
		//Debug.Log (vertBuf.Length);
		lookAtPoint = vertices [vertices.Count - 1];
	}



	void Start () {
		
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
