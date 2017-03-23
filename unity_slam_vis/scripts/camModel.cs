using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class camModel : MonoBehaviour {
	public static Mesh mesh;
	public static Mesh meshBox;
	public static void generateMesh(){
		List<Vector3> verts = new List<Vector3> ();
		List<Color32> colors = new List<Color32> ();
		List<int> inds = new List<int> ();

		float w = 1.0f;
		float h = w*0.75f;
		float z = w*0.6f;

		verts.Add (new Vector3 (0,0,0));
		verts.Add (new Vector3 (w,h,z));
		verts.Add (new Vector3 (0,0,0));
		verts.Add (new Vector3 (w,-h,z));
		verts.Add (new Vector3 (0,0,0));
		verts.Add (new Vector3 (-w,-h,z));
		verts.Add (new Vector3 (0,0,0));
		verts.Add (new Vector3 (-w,h,z));

		verts.Add (new Vector3 (w,h,z));
		verts.Add (new Vector3 (w,-h,z));

		verts.Add (new Vector3 (-w,h,z));
		verts.Add (new Vector3 (-w,-h,z));

		verts.Add (new Vector3 (-w,h,z));
		verts.Add (new Vector3 (w,h,z));

		verts.Add (new Vector3 (-w,-h,z));
		verts.Add (new Vector3 (w,-h,z));

		for (int i = 0; i < 16; i++) {
			inds.Add (i);
			colors.Add (new Color32 (0, 0, 0, 255));
		}

		mesh = new Mesh ();
		mesh.SetVertices (verts);
		mesh.SetColors (colors);
		mesh.SetIndices (inds.ToArray (), MeshTopology.Lines, 0);

		generateMeshBox ();
	}

	static void generateMeshBox(){
		List<Vector3> verts = new List<Vector3> ();
		List<Color32> colors = new List<Color32> ();
		List<int> inds = new List<int> ();

		float w = 0.3f;

		verts.Add (new Vector3 (w,w,w));
		verts.Add (new Vector3 (w,-w,w));
		verts.Add (new Vector3 (w,-w,w));
		verts.Add (new Vector3 (-w,-w,w));
		verts.Add (new Vector3 (-w,-w,w));
		verts.Add (new Vector3 (-w,w,w));
		verts.Add (new Vector3 (-w,w,w));
		verts.Add (new Vector3 (w,w,w));

		verts.Add (new Vector3 (w,w,-w));
		verts.Add (new Vector3 (w,-w,-w));
		verts.Add (new Vector3 (w,-w,-w));
		verts.Add (new Vector3 (-w,-w,-w));
		verts.Add (new Vector3 (-w,-w,-w));
		verts.Add (new Vector3 (-w,w,-w));
		verts.Add (new Vector3 (-w,w,-w));
		verts.Add (new Vector3 (w,w,-w));

		verts.Add (new Vector3 (w,w,w));
		verts.Add (new Vector3 (w,w,-w));
		verts.Add (new Vector3 (w,-w,w));
		verts.Add (new Vector3 (w,-w,-w));
		verts.Add (new Vector3 (-w,-w,w));
		verts.Add (new Vector3 (-w,-w,-w));
		verts.Add (new Vector3 (-w,w,w));
		verts.Add (new Vector3 (-w,w,-w));

		for (int i = 0; i < 24; i++) {
			inds.Add (i);
			colors.Add (new Color32 (0, 0, 0, 255));
		}

		meshBox = new Mesh ();
		meshBox.SetVertices (verts);
		meshBox.SetColors (colors);
		meshBox.SetIndices (inds.ToArray (), MeshTopology.Lines, 0);
	}

	public static camModel create(Vector3 pos, Vector3 right, Vector3 forward, Vector3 up, int id, int _listId){
		GameObject obj = new GameObject ();
		obj.name ="coorMarkCam";

		if (forward.sqrMagnitude <= 0.01) {
			obj.AddComponent<MeshFilter> ().mesh = meshBox;
		}else{
			obj.AddComponent<MeshFilter> ().mesh = mesh;
			Matrix4x4 temp = new Matrix4x4();
			temp.SetColumn (0, new Vector4 (right.x, right.y, right.z, 0));
			temp.SetColumn (1, new Vector4 (up.x, up.y, up.z, 0));
			temp.SetColumn (2, new Vector4 (forward.x, forward.y, forward.z, 0));
			temp.SetColumn (3, new Vector4 (0, 0, 0, 1));
			Quaternion quat = util.QuaternionFromMatrix (temp);
			obj.transform.eulerAngles = quat.eulerAngles;
		}

		Shader mShader = Resources.Load ("pointShader") as Shader;
		obj.AddComponent<MeshRenderer> ().material.shader = mShader;

		obj.transform.position = pos;
		camModel mark = obj.AddComponent<camModel> ();
		BoxCollider collider = obj.AddComponent<BoxCollider> ();
		collider.center =new Vector3(0,0,0.0f);
		collider.size = new Vector3 (0.6f, 0.6f, 0.6f);
		mark.keyFrameId = id;
		mark.listId = _listId;


		return mark;
	}

	public void ChangeModel(int type){
		if (type == 0) {
			GetComponent<MeshFilter> ().mesh = mesh;
		} else if (type == 1) {
			GetComponent<MeshFilter> ().mesh = meshBox;
		}
	}

	public void ChangeColor(Color32 color){
		Mesh meshTemp = GetComponent<MeshFilter> ().mesh;
		Color32[] newColor = new Color32[meshTemp.colors32.Length];
		for (int i = 0; i < meshTemp.colors32.Length; i++) {
			newColor [i] = color;
		}
		meshTemp.colors32 = newColor;
	}


	public int keyFrameId;
	public int listId;

	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
