using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.Collections.Generic;
using System;

public class ChannelInfo: MonoBehaviour{
	public Transform obj;
	public string name;
	public int type;
	public int count;
	public int id; //channel
	public int ListId;
	GameObject menuObj;
	public Toggle showTog;
	public Toggle chooseTog;
	public Toggle redTog;
	public Toggle greenTog;
	public Toggle blueTog;
	public Button lookAtBtn;
	Button cancelBtn;
	public List<mapPoint> MPList;
	public int showCount;
	public List<RoadSeg> RdSegData;

	public bool lineModeTog;
	public bool connectLineTog;
	public bool scaleTog;
	public bool hightlightTog = false;
	public Color32 lineColor;
	public Color32 hightColor;
	public Color32 connectColor;
	public int connectMin;
	public int connectMax;
	public int highlightMin;
	public int highlightMax;
	public int scaleVal;
	public int filterType;
	public int con_type;

	void Start () {
		showCount = -1;
		lineModeTog = false;
		connectLineTog = false;
		scaleTog = false;
		hightlightTog = false;
		lineColor = new Color32(0,0,255,255);
		hightColor = new Color32(255,0,0,255);
		connectColor = new Color32(0,255,0,255);
		connectMin = -1;
		connectMax = 9999;
		highlightMin = -1;
		highlightMax = 9999;
		scaleVal = 100;
		filterType = 0;
		con_type = 0;
	}

	static void HideAllMenu(){
		for (int i = 0; i < commonObj.listRoot.childCount; i++) {
			Transform child = commonObj.listRoot.GetChild (i);
			child.GetComponent<ChannelInfo> ().ShowMenu (false);
		}
	}

	void onChoosed(bool isTrue){
		commonObj.objSettingUI.Show (isTrue, this);
	}

	void onClickItem(){
		if (menuObj.activeSelf == false) {
			HideAllMenu ();
			ShowMenu (true);
		} else {
			ShowMenu (false);
		}
	}

	void LookAt(){
		if (obj == null) {
			return;
		}
		if (obj.gameObject.activeSelf) {
			GameObject camObj = GameObject.Find ("Main Camera") as GameObject;
			CameraMovement cameraCont = camObj.GetComponent<CameraMovement> ();
			if (type == 1) {
				pointCloud cloudObj = obj.GetComponent<pointCloud> ();
				cameraCont.CloseWatch (cloudObj.GetCenterPos ());
			}
			if (type == 2) {
				KFListObj kfObj = obj.GetComponent<KFListObj> ();
				cameraCont.CloseWatch (kfObj.GetCenterPos ());
			}
			if (type == 3) {
				RoadMesh raodobj = obj.GetComponent<RoadMesh> ();
				cameraCont.CloseWatch (raodobj.lookAtPoint);
			}
		}
	}

	public static ChannelInfo getChannel(int listId){
		for (int j = 0; j < commonObj.listRoot.childCount; j++) {
			Transform child = commonObj.listRoot.GetChild (j);
			ChannelInfo item = child.GetComponent<ChannelInfo> ();
			if (item.ListId == listId) {
				return item;
			}
		}
		return null;
	}

	public static ChannelInfo getChannel(int type, int channelId){
		int listId = type * setting.MaxChannel + channelId;
		return getChannel (listId);

	}


	public void DelObj(){
		if (obj!=null) {
			GameObject.Destroy (obj.gameObject);
			obj = null;
		}
	}

	public void onAdjustParam(){
		if (obj == null || !obj.gameObject.activeSelf) {
			return;
		}

		if (type == 1) {
			if (hightlightTog) {
				obj.GetComponent<pointCloud> ().ChangeColor (hightColor, highlightMin, highlightMax, filterType);
			} else {
				//obj.GetComponent<pointCloud> ().ChangeColor (getColorFromUI(), -1, 99999, 0);
			}
			if (scaleTog) {
				obj.GetComponent<pointCloud> ().ChangeScale (scaleVal);
			} else {
				//obj.GetComponent<pointCloud> ().ChangeScale (100);
			}
		}
		if (type == 2) {
			List<KeyFrameC> kfList = CppInterface.GetKFList (id);
			if (hightlightTog) {
				obj.GetComponent<KFListObj> ().ChangeColors (hightColor , highlightMin, highlightMax, filterType);
			} else {
				//obj.GetComponent<KFListObj> ().ChangeColors (getColorFromUI() , -1, 99999, filterType);
			}
			obj.GetComponent<KFListObj> ().ShowTraj(lineModeTog, lineColor);
			if (connectLineTog) {
				obj.GetComponent<KFListObj> ().ClearKFConnection();
				for (int i = 0; i < kfList.Count; i++) {
					ShowKFConnect(i);
				}
			} else {
				obj.GetComponent<KFListObj> ().ClearKFConnection();
			}
			if (scaleTog) {
				obj.GetComponent<KFListObj> ().changeScale (scaleVal);
			} else {
				//obj.GetComponent<KFListObj> ().changeScale (100);
			}
		}
	}

	public void ShowKFConnect(int kfid){
		if (obj.GetComponent<KFListObj> () != null) {
			obj.GetComponent<KFListObj> ().ConnectKF (connectMin, connectMax, kfid, con_type, connectColor);
		}

	}

	public void ShowKFInfo(int kfid){
		if (obj.GetComponent<KFListObj> () != null) {
			string infoStr = obj.GetComponent<KFListObj> ().GetKFInfo(kfid);
			commonObj.mMainUI.debugLogUI.text = infoStr + commonObj.mMainUI.debugLogUI.text;
		}

	}

	public void ShowMPConnect(int kfid){
		if (obj.GetComponent<KFListObj> () != null) {
			ChannelInfo item = getChannel (1, id);
			if (item != null) {
				if (item.obj==null) {
					return;
				}
				if (item.obj.gameObject.activeSelf==false) {
					return;
				}
				obj.GetComponent<KFListObj> ().ConnectMP (item.obj.GetComponent<pointCloud> ().pointlist, kfid, new Color32 (255, 255, 255, 255));
			}
		}
	}

	public void ShowObj(bool val, int scale = 100){
		if (menuObj == null && val == true) {
			initMenu ();
		}
		if (obj==null && val == true) {
			if (type == 1) {
				obj = pointCloud.create (CppInterface.GetMPList (id));
			}
			if (type == 2) {
				obj = KFListObj.create (CppInterface.GetKFList (id), ListId).transform;
			}
			if (type == 3) {
				RdSegData = CppInterface.GetRSList (id);
				if (showCount > RdSegData.Count || showCount<=0) {
					showCount = RdSegData.Count;
				}
				obj = RoadMesh.create (RdSegData.GetRange(0,showCount)).transform;
			}
			if (type == 5) {
				ChannelInfo img = ChannelInfo.getChannel (5, id);
				ChannelInfo feature = ChannelInfo.getChannel (4, id);
				if (img != null) {
					commonObj.mMainUI.updateFrameDisplay (img, feature);
				}
			}
			if (type == 6) {
				List<TraficSign> tmp = CppInterface.GetTSList (id);
				if (showCount >=0 && showCount < tmp.Count) {
					tmp.RemoveRange (showCount,tmp.Count-showCount);
				}
				obj = TraficSignMesh.create (tmp).transform;
			}
			if (type == 7) {
				List<ActMapPoint> tmp = CppInterface.GetAMPList (id);
				obj = ActMPMesh.create (tmp);
			}
		}
		if (obj != null) {
			obj.gameObject.SetActive (val);
			if (val) {
				transform.Find ("Text").GetComponent<Text> ().color = getColorFromUI();
			} else {
				transform.Find ("Text").GetComponent<Text> ().color = new Color32 (0, 0, 0, 255);
			}
		}
	}

	Color32 getColorFromUI(){
		byte r = 0, g = 0, b = 0;
		if (redTog.isOn) {
			r = 255;
		}
		if (greenTog.isOn) {
			g = 255;
		}
		if (blueTog.isOn) {
			b = 255;
		}
		return new Color32 (r, g, b, 255);
	}

	void updateMPPointColor(){
		if (type == 1) {
			obj.GetComponent<pointCloud> ().ChangeColor (getColorFromUI(), -1, Int32.MaxValue, 0);
		}else if(type == 2){
			obj.GetComponent<KFListObj> ().ChangeColors (getColorFromUI() , -1, Int32.MaxValue, filterType);
		}
//		else if(type == 7){
//			obj.GetComponent<ActMPMesh> ().ChangeColors (getColorFromUI() , -1, Int32.MaxValue, filterType);
//		}
	}

	public void initMenu(){
		if (menuObj != null) {
			return;
		}
		menuObj = Instantiate (commonObj.menuPrefab,Vector3.zero, Quaternion.identity) as GameObject;
		menuObj.transform.SetParent (transform,false);
		menuObj.GetComponent<RectTransform> ().localScale = new Vector3 (1, 1, 1);
		menuObj.SetActive (false);
		chooseTog = menuObj.transform.FindChild ("Choose").GetComponent<Toggle> ();
		redTog = menuObj.transform.FindChild ("Red").GetComponent<Toggle> ();
		greenTog = menuObj.transform.FindChild ("Green").GetComponent<Toggle> ();
		blueTog = menuObj.transform.FindChild ("Blue").GetComponent<Toggle> ();
		chooseTog.onValueChanged.AddListener ((bool arg1) => onChoosed(arg1));
		chooseTog.isOn = false;
		redTog.onValueChanged.AddListener ((bool arg1) => updateMPPointColor());
		greenTog.onValueChanged.AddListener ((bool arg1) => updateMPPointColor());
		blueTog.onValueChanged.AddListener ((bool arg1) => updateMPPointColor());

		showTog = menuObj.transform.FindChild("Show").GetComponent<Toggle>();
		showTog.onValueChanged.AddListener ((bool arg1)=>ShowObj(arg1));
		showTog.isOn = false;

		lookAtBtn = menuObj.transform.FindChild("LookAt").GetComponent<Button>();
		lookAtBtn.onClick.AddListener (() => LookAt ());
		cancelBtn = menuObj.transform.FindChild("Cancel").GetComponent<Button>();
		cancelBtn.onClick.AddListener (() => ShowMenu(false));
		
	}

	public void ShowMenu(bool val){
		if (menuObj == null) {
			if (val == false) {
				return;
			}
			initMenu ();
		}
		if (menuObj != null) {
			menuObj.SetActive (val);
			commonObj.objSettingUI.Show (chooseTog.isOn, this);
		}

	}

	public static ChannelInfo create(ChannelInfoRaw item){
		int channelUIId = item.type * setting.MaxChannel + item.id;
		GameObject clone;
		if (true) {
			clone = Instantiate (commonObj.ListItemPrefab, Vector3.zero, Quaternion.identity) as GameObject;
			clone.transform.Find ("Text").GetComponent<Text> ().text = 
				setting.DateTypeName[item.type]+
				"("+ item.id+")"+
				":"+ item.count;
			ChannelInfo channelTemp = clone.GetComponent<ChannelInfo> ();
			clone.GetComponents<Button> () [0].onClick.AddListener(
				()=>channelTemp.onClickItem());
		} else {
			clone = new GameObject ();
			clone.AddComponent<ChannelInfo> ();
		}

		ChannelInfo channel = clone.GetComponent<ChannelInfo> ();
		channel.id = item.id;
		channel.type = item.type;
		channel.name = item.name;
		channel.count = item.count;
		channel.ListId = channelUIId;
		clone.name = channelUIId.ToString();
		clone.transform.SetParent(commonObj.listRoot,false);
		channel.initMenu ();
		return channel;
	}

	public void OnDestroy(){
		if (obj != null) {
			Destroy (obj.gameObject);
		}

		if (menuObj != null) {
			Destroy (menuObj);
		}
	}
};
