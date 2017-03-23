using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.Collections.Generic;

public class CarC{
	
	public static int curCarId = 0;
	public static List<CarC> carList;

	private static GameObject carListUI;
	private static GameObject carItemUIFab;
	private static CameraMovement cameraCont;

	public static CarC create (List<KeyFrameC> carKFLists, List<KeyFrameC> sCarKFLists, List<mapPoint> carMPLists, List<mapPoint> sCarMPLists){
		if (carItemUIFab == null) {
			carItemUIFab = Resources.Load ("carItemUI") as GameObject;
			carListUI = GameObject.Find ("Canvas/Panel/CarList/ScrollView/List");
			carList = new List<CarC>();
			GameObject obj = GameObject.Find ("Main Camera") as GameObject;
			cameraCont = obj.GetComponents<CameraMovement> () [0];
		}
		if (carKFLists.Count > 0) {
			GameObject clone = Object.Instantiate(carItemUIFab,
				Vector3.zero, 
				Quaternion.identity
			) as GameObject;
			clone.transform.Find ("Text").GetComponents<Text> () [0].text = 
				"Car " + curCarId;
			CarC itemCar = new CarC ();
			clone.GetComponents<Button> () [0].onClick.AddListener(
				()=>itemCar.onClickCarUI());
			itemCar.btn = clone.transform;
			itemCar.id = curCarId;
			curCarId= curCarId+1;
			clone.transform.SetParent(carListUI.transform);
			KFListObj carKF = KFListObj.create (carKFLists, 0);
			itemCar.keyFrameObj = carKF;
			carList.Add (itemCar);
			return itemCar;
		}
		return null;
	}


	public int id;
	public List<mapPoint> mapPointList;
	public KFListObj keyFrameObj;
	public Transform btn;
	public CarC(){
		mapPointList = new List<mapPoint> ();
	}
	void onClickCarUI(){
		cameraCont.CloseWatch (keyFrameObj.GetCenterPos ());
	}
}


public class KFListObj : MonoBehaviour {
	public static Transform rootObj;

	public static void ClearAll(){
		if (rootObj != null) {
			GameObject.Destroy (rootObj.gameObject);
			rootObj = null;
		}
	}

	public static KFListObj create (List<KeyFrameC> kFList, int _listId){
		if (rootObj==null) {
			rootObj = new GameObject ().transform;
			rootObj.name= "KFRoot";
		}
		GameObject  obj = new GameObject ();
		obj.transform.SetParent (rootObj);
		obj.name ="KFList";
		KFListObj kfListObj = obj.AddComponent<KFListObj> ();
		kfListObj.KFList = kFList;
		kfListObj.listId = _listId;
		return kfListObj;
	}
		
	Transform kfMarkRoot;
	Transform kfTrajRoot;
	Transform kfConnectionRoot;
	Transform mpConnectionRoot;
	public int listId;
	public List<KeyFrameC> KFList;
	void Start () {
		kfMarkRoot = new GameObject ().transform;
		kfConnectionRoot = new GameObject ().transform;
		mpConnectionRoot = lineMesh.create ().transform;
		mpConnectionRoot.SetParent (transform);
		kfConnectionRoot.SetParent (transform);
		kfMarkRoot.SetParent (transform);
		lineMesh trajObj = lineMesh.create ();
		kfTrajRoot = trajObj.transform;
		kfTrajRoot.SetParent (transform);
		for (int i = 0; i < KFList.Count; i++) {
			camModel mark = camModel.create (KFList[i].pos, 
				KFList[i].right, 
				KFList[i].forward, 
				KFList[i].up, 
				KFList[i].id, listId);
			mark.ChangeColor (KFList[i].color);
			mark.transform.SetParent (kfMarkRoot);
			if (i != 0) {
				trajObj.addLine (KFList [i - 1].pos, KFList [i].pos);

			}
		}
		trajObj.createMesh ();
		//ShowTraj (false, new Color32());
	}

	public void changeScale(int scale){
		kfTrajRoot.GetComponent<lineMesh> ().changeScale (scale);

		for (int i=0; i<KFList.Count; i++){
			KeyFrameC item = KFList [i];
			item.pos = item.pos * scale / 100.0f;
			KFList [i] = item;
			Transform child = kfMarkRoot.GetChild (i);
			child.position = KFList [i].pos;
		}

		ClearKFConnection ();
	}

	public string GetKFInfo(int kfId){
		int ind = findKFInd (kfId);
		KeyFrameC item = KFList [ind];
		if (item == null) {
			Debug.LogError ("keyframe id: " + kfId + "not found!");
			return "";
		}

		string re = "kfId:" + item.id + " pos:" + item.pos.ToString ();
		re = re + " filter1:" + item.filter1 + " filter2:" + item.filter2 + " filter3:" + item.filter3 + '\n';
		if (item.kfLinks1 != null) {
			re = re + " linkVal1: ";
			for (int i = 0; i < item.kfLinks1.Length; i++) {
				re = re + "[" + item.kfLinks1 [i] + ":" + item.kfLinksVal1 [i] + "]";
			}
			re= re+'\n';
		}
		if (item.kfLinks2 != null) {
			re = re + " linkVal2: ";
			for (int i = 0; i < item.kfLinks2.Length; i++) {
				re = re + "[" + item.kfLinks2 [i] + ":" + item.kfLinksVal2 [i] + "]";
			}
			re= re+'\n';
		}
		if (item.kfLinks3 != null) {
			re = re + " linkVal3: ";
			for (int i = 0; i < item.kfLinks3.Length; i++) {
				re = re + "[" + item.kfLinks3 [i] + ":" + item.kfLinksVal3 [i] + "]";
			}
			re= re+'\n';
		}
		re= re+"================================\n";
		return re;
	
	}

	public void ConnectKF(int minVal, int maxVal, int kfId, int con_type, Color32 color){
		int ind = findKFInd (kfId);
		int[] list;
		int[] listVal;
		if (con_type == 0) {
			list = KFList [ind].kfLinks1;
			listVal = KFList [ind].kfLinksVal1;
		} else if (con_type == 1) {
			list = KFList [ind].kfLinks2;
			listVal = KFList [ind].kfLinksVal2;
		} else if (con_type == 2) {
			list = KFList [ind].kfLinks3;
			listVal = KFList [ind].kfLinksVal3;
		} else {
			return;
		}
		if (list == null) {
			Debug.LogError ("There is no data for this connection of keyframe. Id:"+ kfId);
			return;
		}
		lineMesh tempObj = lineMesh.create ();
		tempObj.transform.SetParent (kfConnectionRoot);
		tempObj.color = color;
		for (int i = 0; i < list.Length; i++) {
			if (maxVal >= listVal [i] && minVal <= listVal [i]) {
				int kfInd = findKFInd (list [i]);
				if (kfInd != -1) {
					KeyFrameC item = KFList [kfInd];
					GetMarkObj (kfInd).ChangeColor (color);
					tempObj.addLine (KFList [ind].pos, item.pos);
				} else {
					Debug.LogError ("keyframe "+list [i] +"is not found!!");
						
				}
			}
		}
		tempObj.createMesh ();
	}

	mapPoint findMP(List<mapPoint> mplist, int tarId){
		for (int i = 0; i < mplist.Count; i++) {
			if (mplist [i].id == tarId) {
				return mplist [i];
			}
		}
		return null;
	}

	KeyFrameC findKF(int tarId){
		for (int i = 0; i < KFList.Count; i++) {
			if (KFList [i].id == tarId) {
				return KFList [i];
			}
		}
		return null;
	}

	int findKFInd(int tarId){
		for (int i = 0; i < KFList.Count; i++) {
			if (KFList [i].id == tarId) {
				return i;
			}
		}
		return -1;
	}

	public void ConnectMP(List<mapPoint> mplist, int kfId, Color32 color){
		if (mplist == null) {
			return;
		}
		int ind = findKFInd (kfId);
		int[] mask = KFList [ind].mpLinks;
		bool isShow = !mpConnectionRoot.gameObject.activeSelf;
		if (isShow) {
			mpConnectionRoot.GetComponent<lineMesh> ().clearAllLine ();
			mpConnectionRoot.GetComponent<lineMesh> ().color = color;
			for (int i = 0; i < mask.Length; i++) {
				mapPoint item = findMP (mplist, mask[i]);
				if (item != null) {
					mpConnectionRoot.GetComponent<lineMesh> ().addLine (KFList[ind].pos, item.pos);
				}
			}
			mpConnectionRoot.GetComponent<lineMesh> ().createMesh ();
		}
		mpConnectionRoot.gameObject.SetActive (isShow);

	}

	public void ClearKFConnection(){
		for (int i = 0; i < kfConnectionRoot.transform.childCount; i++) {
			Transform objChild = kfConnectionRoot.transform.GetChild (i);
			Destroy(objChild.gameObject);
		}
		kfConnectionRoot.transform.DetachChildren ();
	}

	public void ShowTraj(bool isTrue, Color32 color){
		if (isTrue) {
			kfMarkRoot.gameObject.SetActive (false);
			//kfTrajRoot.gameObject.SetActive (true);
			kfTrajRoot.GetComponent<lineMesh> ().ChangeColor (color);
		} else {
			kfMarkRoot.gameObject.SetActive (true);
			//kfTrajRoot.gameObject.SetActive (false);
		}
	}

	public camModel GetMarkObj(int id){
		return kfMarkRoot.GetChild (id).GetComponent<camModel> ();
	}

	public void ChangeColors(Color32 hightColor, int highlightMin, int highlightMax, int filterType){
		for (int i = 0; i < KFList.Count; i++) {
			int val = 0;
			if (filterType == 0) {
				val = KFList [i].filter1;
			} else if (filterType == 1) {
				val = KFList [i].filter2;
			} else if (filterType == 2) {
				val = KFList [i].filter3;
			}
			if (highlightMax >= val && highlightMin <= val) {
				GetMarkObj (i).ChangeColor (hightColor);
			}
		}
	}

	public Vector3 GetCenterPos(){
		Vector3 tempCentPos = new Vector3(0,0,0);
		for (int i = 0; i < KFList.Count; i++) {
			tempCentPos = tempCentPos + KFList [i].pos;
		}
		if (KFList.Count > 0) {
			tempCentPos = tempCentPos / KFList.Count;
		}
		return tempCentPos;
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
