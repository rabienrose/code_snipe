using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class tryRotation : MonoBehaviour {

	// Use this for initialization
	void Start () {
		GameObject[] objList = new GameObject[4];
		objList [0] = GameObject.Find ("Capsule") as GameObject;
		objList [1] = GameObject.Find ("Capsule1") as GameObject;
		objList [2] = GameObject.Find ("Capsule2") as GameObject;
		objList [3] = GameObject.Find ("Capsule3") as GameObject;
		Quaternion rot90q = Quaternion.Euler(new Vector3(0,-60,0));
		Matrix4x4 rot90m = Matrix4x4.TRS (new Vector3 (0, 0, 10), rot90q, new Vector3 (1, 1, 1));
		Debug.Log (rot90q);
		Debug.Log (rot90m);

		for(int i=0;i<4;i++){
			Matrix4x4 totalMat = rot90m * objList [i].transform.localToWorldMatrix;
			FromMatrix4x4 (objList [i].transform, totalMat);
		}

	
	}
	
	// Update is called once per frame
	void Update () {
	
	}

	public void FromMatrix4x4(Transform transform, Matrix4x4 matrix)
	{
		transform.localScale = GetScale(matrix);
		transform.rotation = GetRotation(matrix);
		transform.position = GetPosition(matrix);
	}

	public Quaternion GetRotation(Matrix4x4 matrix)
	{
		var qw = Mathf.Sqrt(1f + matrix.m00 + matrix.m11 + matrix.m22) / 2;
		var w = 4 * qw;
		var qx = (matrix.m21 - matrix.m12) / w;
		var qy = (matrix.m02 - matrix.m20) / w;
		var qz = (matrix.m10 - matrix.m01) / w;

		return new Quaternion(qx, qy, qz, qw);
	}

	public Vector3 GetPosition(Matrix4x4 matrix)
	{
		var x = matrix.m03;
		var y = matrix.m13;
		var z = matrix.m23;

		return new Vector3(x, y, z);
	}

	public Vector3 GetScale(Matrix4x4 m)
	{
		var x = Mathf.Sqrt(m.m00 * m.m00 + m.m01 * m.m01 + m.m02 * m.m02);
		var y = Mathf.Sqrt(m.m10 * m.m10 + m.m11 * m.m11 + m.m12 * m.m12);
		var z = Mathf.Sqrt(m.m20 * m.m20 + m.m21 * m.m21 + m.m22 * m.m22);

		return new Vector3(x, y, z);
	}
}
