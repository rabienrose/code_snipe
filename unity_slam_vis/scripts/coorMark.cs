using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class coorMark : MonoBehaviour {

	// Use this for initialization
	public static coorMark create(Vector3 pos, Vector3 right, Vector3 forward, Vector3 up, int id){
		GameObject obj = new GameObject ();
		obj.name ="coorMark";
		Mesh mesh = new Mesh ();
		List<Vector3> vertice = new List<Vector3> ();
		List<Color32> colors = new List<Color32> ();
		List<int> inds = new List<int> ();
		float scaleMark = 1f;
		vertice.Add (new Vector3 (0, 0, 0));
		colors.Add (new Color32 (255, 0, 0, 255));
		vertice.Add (new Vector3 (scaleMark, 0, 0));//right
		colors.Add (new Color32 (255, 0, 0, 255));
		vertice.Add (new Vector3 (0, 0, 0));
		colors.Add (new Color32 (0, 255, 0, 255));
		vertice.Add (new Vector3 (0, scaleMark, 0));//up
		colors.Add (new Color32 (0, 255, 0, 255));
		vertice.Add (new Vector3 (0, 0, 0));
		colors.Add (new Color32 (0, 0, 255, 255));
		vertice.Add (new Vector3 (0, 0, scaleMark));//forward
		colors.Add (new Color32 (0, 0, 255, 255));

		inds.Add (0);
		inds.Add (1);
		inds.Add (2);
		inds.Add (3);
		inds.Add (4);
		inds.Add (5);

		mesh.SetVertices (vertice);
		mesh.SetColors (colors);
		mesh.SetIndices (inds.ToArray(), MeshTopology.Lines, 0);
		Shader mShader = Resources.Load ("pointShader") as Shader;
		obj.AddComponent<MeshFilter> ().mesh = mesh;
		obj.AddComponent<MeshRenderer> ().material.shader = mShader;
		coorMark mark = obj.AddComponent<coorMark> ();
		BoxCollider collider = obj.AddComponent<BoxCollider> ();
		//Debug.LogError (forward);
		//obj.transform.right = right;
		//obj.transform.forward = forward;
		//obj.transform.up = up;
		Matrix4x4 temp = new Matrix4x4();
		temp.SetColumn (0, new Vector4 (right.x, right.y, right.z, 0));
		temp.SetColumn (1, new Vector4 (up.x, up.y, up.z, 0));
		temp.SetColumn (2, new Vector4 (forward.x, forward.y, forward.z, 0));
		temp.SetColumn (3, new Vector4 (0, 0, 0, 1));
		Quaternion quat = util.QuaternionFromMatrix (temp);
		obj.transform.eulerAngles = quat.eulerAngles;


		obj.transform.position = pos;
		collider.center =new Vector3(0,0,0);
		collider.size = new Vector3 (0.2f, 0.2f, 0.2f);
		mark.keyFrameId = id;
		return mark;
	}


	public int keyFrameId;
	public pointCloud cloud;

	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
