using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class meshHander : MonoBehaviour {
	float trajR=10;
	float camHeight=2;
	float roadHW=1;

	List<Vector3> camPosList = new List<Vector3> ();
	List<float> roadHList = new List<float> ();
	List<float> roadWRList = new List<float> ();
	List<float> roadWLList = new List<float> ();
	List<Vector3> roadNormalList = new List<Vector3> ();

	//MapPointMgr pointMgr;

	// Use this for initialization
	void Start () {
		//read data from lib
		//int aa = AddTwoIntegers(2,3);
		//Debug.LogError(aa);

//		int nodeCount = CppInterface.readExistedSegData ();
//		//Debug.LogError (nodeCount);
//		for (int i = 0; i < nodeCount; i++) {
//			RoadSeg item = CppInterface.getSegData (i);
//			camPosList.Add (new Vector3 (-item.camX, -item.camY, -item.camZ));
//			roadHList.Add (-item.height);
//			if (item.hWidthR > 4) {
//				item.hWidthR = roadWRList [i - 1];
//			}
//			if (item.hWidthL > 4) {
//				item.hWidthL = roadWLList [i - 1];
//			}
//			roadWRList.Add (item.hWidthR);
//			roadWLList.Add (item.hWidthL);
//			roadNormalList.Add (new Vector3 (-item.normX, -item.normY, -item.normZ));
//		}
//
//		List<Vector3> roadP1List = new List<Vector3> ();
//		List<Vector3> roadP2List = new List<Vector3> ();
//
//		for (int i = 0; i < nodeCount-1; i++) {
//			Vector3 trajVec = camPosList [i+1] - camPosList [i];
//			Vector3 roadWidthVec = Vector3.Cross (trajVec, roadNormalList [i]);
//			roadWidthVec.Normalize ();
//			Vector3 roadC = new Vector3 (camPosList [i].x, camPosList [i].y + roadHList [i], camPosList [i].z);
//			Vector3 roadEdgeP1 = roadC + roadWidthVec * roadWRList[i];
//			Vector3 roadEdgeP2 = roadC + -roadWidthVec * roadWLList[i];
//			roadP1List.Add (roadEdgeP1);
//			roadP2List.Add (roadEdgeP2);
//			coorMark mark = coorMark.create (camPosList [i], roadWidthVec, trajVec, roadNormalList [i], i);
//			int linePointCont = CppInterface.getLinePointsCount (i);
//			List<mapPoint> mapPointList = new List<mapPoint> ();
//			for (int j = 0; j < linePointCont; j++) {
//				RoadPoint p = CppInterface.getLinePoint (i, j);
//				mapPoint mp = new mapPoint();
//				mp.Id = -1;
//				mp.pos = new Vector3 (-p.x, -p.y, -p.z);
//				mp.color = new Color32 ((byte)p.r, (byte)p.g, (byte)p.b, 255);
//				mapPointList.Add (mp);
//			}
//			//pointCloud.create (mapPointList);
//			//cloud.hideCloud (true);
//			//mark.cloud = cloud;
//		}
//
//		List<Vector3> vertices = new List<Vector3>();
//		List<Vector2> uv = new List<Vector2>();
//		List<int> triang = new List<int>();
//		vertices.Add (roadP1List [0]);
//		uv.Add (new Vector2 (1f, 0f));
//		vertices.Add (roadP2List [0]);
//		uv.Add (new Vector2 (0f, 1f));
//		for (int i = 0; i < nodeCount-2; i++) {
//			vertices.Add (roadP1List [i+1]);
//			uv.Add (new Vector2 (1f, 0f));
//			vertices.Add (roadP2List [i+1]);
//			uv.Add (new Vector2 (0f, 1f));
//			triang.Add (i * 2+2);
//			triang.Add (i * 2+1);
//			triang.Add (i * 2+0);
//			//double sides displaying
//			triang.Add (i * 2+0);
//			triang.Add (i * 2+1);
//			triang.Add (i * 2+2);
//
//			triang.Add (i * 2+1);
//			triang.Add (i * 2+2);
//			triang.Add (i * 2+3);
//			//double sides displaying
//			triang.Add (i * 2+3);
//			triang.Add (i * 2+2);
//			triang.Add (i * 2+1);
//		}
//		Vector3[] vertBuf = vertices.ToArray ();
//		int[] triangBuf = triang.ToArray();
//		Vector2[] uvBuf = uv.ToArray ();
//		Mesh mesh = new Mesh ();
//		GetComponent<MeshFilter> ().mesh = mesh;
//		mesh.vertices = vertBuf;
//		mesh.triangles = triangBuf;
//		mesh.uv = uvBuf;

	}

	float getSmallPertub(){
		return (Random.value-0.5f) * 0.5f;
	}

	void Update () {
	
	}

	void generateTestData(){
//
//		int nodeCount = 50;
//		for (int i = 0; i < nodeCount; i++) {
//			Vector3 camPos;
//			camPos.x = Mathf.Sin (i / (float)nodeCount * 3.1415f *2)*trajR+getSmallPertub();
//			camPos.z = Mathf.Cos (i / (float)nodeCount * 3.1415f *2)*trajR+getSmallPertub();
//			camPos.y = camHeight + getSmallPertub ();
//			camPosList.Add (camPos);
//			roadHList.Add (-camHeight + getSmallPertub ());
//			roadWRList.Add (roadHW + getSmallPertub ());
//			roadWLList.Add (roadHW + getSmallPertub ());
//			Vector3 roadNormal = new Vector3(getSmallPertub(),10+getSmallPertub(),getSmallPertub());
//			roadNormal.Normalize ();
//			roadNormalList.Add (roadNormal);
//		}
	}

	Bounds calBound(ref List<Vector3> pointList){
		float minX = 1000f, maxX = -1000f;
		float minY = 1000f, maxY = -1000f;
		float minZ = 1000f, maxZ = -1000f;
		for (int i = 0; i < pointList.Count; i++) {
			if (pointList [i].x < minX) {
				minX = pointList [i].x;
			}
			if (pointList [i].x > maxX) {
				maxX = pointList [i].x;
			}
			if (pointList [i].y < minY) {
				minY = pointList [i].y;
			}
			if (pointList [i].y > maxY) {
				maxY = pointList [i].y;
			}
			if (pointList [i].z < minZ) {
				minZ = pointList [i].z;
			}
			if (pointList [i].z > maxZ) {
				maxZ = pointList [i].z;
			}
		}
		Debug.Log (new Vector3 (minX, minY, minZ));
		Debug.Log (new Vector3 (maxX, maxY, maxZ));
		Bounds b = new Bounds ();
		b.SetMinMax (new Vector3 (minX, minY, minZ), new Vector3 (maxX, maxY, maxZ));
		return b;
	}
		
}
