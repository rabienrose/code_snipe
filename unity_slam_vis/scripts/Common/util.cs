using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

public class util {

	public static Quaternion QuaternionFromMatrix(Matrix4x4 m) {
		Quaternion q = new Quaternion();
		q.w = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0,0] + m[1,1] + m[2,2] ) ) / 2; 
		q.x = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0,0] - m[1,1] - m[2,2] ) ) / 2; 
		q.y = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0,0] + m[1,1] - m[2,2] ) ) / 2; 
		q.z = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0,0] - m[1,1] + m[2,2] ) ) / 2; 
		q.x *= Mathf.Sign( q.x * ( m[2,1] - m[1,2] ) );
		q.y *= Mathf.Sign( q.y * ( m[0,2] - m[2,0] ) );
		q.z *= Mathf.Sign( q.z * ( m[1,0] - m[0,1] ) );
		return q;
	}

	public static string GetAviableDir(string rawPath){
		return Path.GetDirectoryName (rawPath);
	}

	public static void SaveHistory(Dictionary<string,List<string>>dropHis, Dictionary<string,string>inputHis){
		IFormatter formatter = new BinaryFormatter();
		Stream stream = new FileStream("MyFile.bin", 
			FileMode.Create, 
			FileAccess.Write, FileShare.None);
		formatter.Serialize(stream, dropHis);
		formatter.Serialize(stream, inputHis);
		stream.Close();
	}

	public static void ReadHistory(out Dictionary<string,List<string>>dropHis,out Dictionary<string,string>inputHis){
		IFormatter formatter = new BinaryFormatter();
		if (File.Exists ("MyFile.bin")) {
			Stream stream = new FileStream("MyFile.bin", 
				FileMode.Open, 
				FileAccess.Read, 
				FileShare.Read);
			dropHis = (Dictionary<string,List<string>>) formatter.Deserialize(stream);
			inputHis = (Dictionary<string,string>) formatter.Deserialize(stream);
			stream.Close();
		} else {
			dropHis = new Dictionary<string, List<string>> ();
			inputHis = new Dictionary<string, string> ();
		}
		
	}

	public static void SaveSetting(Dictionary<string,string>setting){
		IFormatter formatter = new BinaryFormatter();
		Stream stream = new FileStream("MyFileSetting.bin", 
			FileMode.Create, 
			FileAccess.Write, FileShare.None);
		formatter.Serialize(stream, setting);
		stream.Close();
	}

	public static void ReadSetting(out Dictionary<string,string>setting){
		IFormatter formatter = new BinaryFormatter();
		if (File.Exists ("MyFileSetting.bin")) {
			Stream stream = new FileStream("MyFileSetting.bin", 
				FileMode.Open, 
				FileAccess.Read, 
				FileShare.Read);
			setting = (Dictionary<string,string>) formatter.Deserialize(stream);
			stream.Close();
		} else {
			setting = new Dictionary<string, string> ();
		}

	}


}
