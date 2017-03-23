using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using OpenCVForUnity;
using SkywardRay.FileBrowser;
using System;
using System.Threading;
using System.IO;
using System.Security.Permissions;

public class uiMain : MonoBehaviour {
	public string FileAddrUI;
	public string[] Params = new string[4];
	public int MaxStepUI;
	public int NextStepUI;
	public int AlgoTypeUI;
	public int FileTypeUI;

	Transform mainPanel;

	public Dictionary<string,List<string>> inputBoxHistory;
	public Dictionary<string,string> lastOpenHistory;
	public Dictionary<string,string> uiSetting;
	public GameObject prefabBrowser;

	string openFileRecordName = "chamo";
	int nextActionId;
	private string[] extensions = { "chamo"};

	public GameObject listUI;
	public Transform FrameDisplay;

	InputField curChennel;
	Thread oThread;

	Button runBtn;
	Button stopBtn;
	Button reloadBtn;
	Button clearLogBtn;
	public Text debugLogUI;
	Toggle showFrameUI;
	Toggle showLogUI;
	InputField GScaleUI;

	enum UIState {idle, running, stepping};
	UIState curState;
	string curOutputFile;

	float updateCurTime;

	bool requestPause;


	void Awake () {
		
	}
	void Start () {
		MaxStepUI =0;
		NextStepUI=-1;
		AlgoTypeUI = 1;
		FileTypeUI = 1;
		commonObj.Init ();
		commonObj.mMainUI = this;
		curChennel = transform.FindChild ("Panel/ChannelId").GetComponent<InputField>();
		curChennel.text = "0";
		CppInterface.NewViewHandle ();
		util.ReadHistory (out inputBoxHistory, out lastOpenHistory);
		util.ReadSetting (out uiSetting);
		mainPanel = transform.FindChild ("Panel");
		updateListUI ();
		commonObj.prefabFileBrower = prefabBrowser;
		listUI = GameObject.Find ("Canvas/Panel/ListRoot/ScrollView/List");
		commonObj.listRoot = listUI.transform;
		runBtn= GameObject.Find ("Canvas/Panel/RunBtn").GetComponent<Button>();
		stopBtn= GameObject.Find ("Canvas/Panel/Stop").GetComponent<Button>();
		reloadBtn= GameObject.Find ("Canvas/Panel/Reload").GetComponent<Button>();
		clearLogBtn= GameObject.Find ("Canvas/Panel/ClearLog").GetComponent<Button>(); 
		debugLogUI = GameObject.Find ("Canvas/DebugLog").GetComponent<Text>();
		//GameObject.Find ("Canvas/DebugLog").SetActive (false);
		showFrameUI = GameObject.Find ("Canvas/Panel/HideFrame").GetComponent<Toggle>();
		showLogUI = GameObject.Find ("Canvas/Panel/ShowLog").GetComponent<Toggle>();
		GScaleUI = GameObject.Find ("Canvas/Panel/GScale").GetComponent<InputField>();
		if (uiSetting.ContainsKey ("GScale")) {
			string gscaleStr = uiSetting ["GScale"];
			onChangeGScale (gscaleStr);
			GScaleUI.text = gscaleStr;
		}

		//debugLogUI.color = new Color (255, 0, 0, 255);
		runBtn.interactable = true;
		curState = UIState.idle;
		updateCurTime = 0;
		requestPause = false;
		commonObj.goundGird = groundGird.create ();
		commonObj.goundGird.gameObject.SetActive (true);
		camModel.generateMesh ();
	}

	public void onClearLogBtn(){
		debugLogUI.text = "";
	}

	public void onChangeGScale(string num){
		float tScale =  float.Parse (num);
		CppInterface.scale = tScale;
		//if (!uiSetting.ContainsKey ("GScale")) {
			uiSetting["GScale"] = num;
		//}
		util.SaveSetting (uiSetting);
	}

	public void SaveHistory(){
		util.SaveHistory (inputBoxHistory, lastOpenHistory);
	}
		
	void Update () {
		
		if (curState == UIState.running) {
			updateCurTime = updateCurTime + Time.deltaTime;
			if (updateCurTime > 0.5f) {
				dealWithResult ();
				if (showLogUI.isOn) {
					using (StreamReader sr = new StreamReader("debug.txt"))
					{
						String line = sr.ReadToEnd();
						sr.Close ();
						char[] spliter = {'\n'};
						string[] lines = line.Split (spliter);
						string showLine = "";
						int startLine = 0;
						if (lines.Length - setting.MaxLogLines > 0) {
							startLine = lines.Length - setting.MaxLogLines;
						}
						for (int i = startLine; i < lines.Length; i++) {
							showLine = showLine + lines [i]+'\n';
						}
						debugLogUI.text = showLine;
					}
				}
				updateCurTime = 0;
			} else {
				return;
			}
		}
		else if (curState == UIState.idle) {

		}
	}

	void RunThread(){
		curState = UIState.running;
		CppInterface.InitAlgo (AlgoTypeUI);
		curState = UIState.idle;
	}

	public void onRunBtn(){
		CppInterface.NewViewHandle ();
		if (AlgoTypeUI == 1) {
			CppInterface.SetParam (0, Params[0]);
			CppInterface.SetParam (1, Params[1]);
			CppInterface.SetParam (2, Params[2]);
			CppInterface.SetParam (3, Params[3]);
			FileAddrUI = Params [3];
		} 
		if (AlgoTypeUI == 2) {

		}
		if (AlgoTypeUI == 3) {
			CppInterface.SetParam (0, Params[0]);
			CppInterface.SetParam (1, Params[1]);
		}
		oThread = new Thread (RunThread);
		oThread.Start ();
	}

	[SecurityPermissionAttribute(SecurityAction.Demand, ControlThread = true)]
	public void onStop(){
		Debug.LogError ("try to stop running!");
		oThread.Abort ();
	}

	public void onReload(){
		onReadResultBtn ();
	}

	public void showTerrain(bool val){
		if (val) {
			commonObj.groundTer.gameObject.SetActive (true);
			commonObj.goundGird.gameObject.SetActive (false);
		} else {
			commonObj.groundTer.gameObject.SetActive (false);
			commonObj.goundGird.gameObject.SetActive (true);
		}
	}

	public void updateFrameDisplay(ChannelInfo img, ChannelInfo Feature){
		List<Mat> imgList = CppInterface.GetIMList(img.id);
		Mat colorMat = new Mat();
		Imgproc.cvtColor(imgList[0],colorMat,Imgproc.COLOR_GRAY2RGB);
		if (Feature != null) {
			List<FeaturesCpp> featureList = CppInterface.GetFPList(Feature.id);
			for (int i = 0; i < featureList.Count; i++) {
				FeaturesCpp info = featureList [i];
				Imgproc.circle(colorMat,new Point(info.u, info.v),info.r/2,new Scalar(info.red,info.green,0),2);
				if(info.lastU>10){
					Imgproc.line(colorMat,new Point(info.u, info.v), new Point(info.lastU, info.lastV),new Scalar(255,0,255),2);
				}
			}
		}

		RectTransform sizeSetting = FrameDisplay.GetComponent<RectTransform> ();
		float winH = colorMat.rows ();
		float winW = colorMat.cols ();
		if (winH > 600) {
			winW = winW * 600f / winH;
			winH = 600;

		}
		sizeSetting.sizeDelta = new Vector2( winW, winH);
		Texture2D texture = new Texture2D (colorMat.cols (), colorMat.rows (), TextureFormat.RGBA32, false);
		Utils.matToTexture2D (colorMat, texture);
		FrameDisplay.GetComponent<RawImage> ().texture = texture;
	}

	//list related
	public void removeListUI(int listId){
		for (int i = 0; i < listUI.transform.childCount; i++) {
			Transform child = listUI.transform.GetChild (i);
			if (child.gameObject.name == listId.ToString ()) {
				GameObject.Destroy (child);
			}
		}
	}

	public ChannelInfo addListUI(ChannelInfoRaw item){
		int listID = item.type * setting.MaxChannel + item.id;
		removeListUI (listID);
		return ChannelInfo.create (item);
	}

	public void updateListUI(){
		onClearSceneBtn ();
		List<ChannelInfoRaw> tempList =CppInterface.GetAllChannelInfo();
		for (int i = 0; i < tempList.Count; i++) {
			ChannelInfo.create (tempList[i]);
		}
	}
		
	//data file related
	public void onReadFileBtn(){
		int channelId = Int32.Parse (curChennel.text);
		int count =CppInterface.ReadSingleData (FileAddrUI, FileTypeUI, channelId);
		if (count > 0) {
			ChannelInfoRaw channelItem = new ChannelInfoRaw();
			channelItem.type = FileTypeUI; 
			channelItem.id = channelId;
			channelItem.count = count;
			ChannelInfo uiItem = addListUI (channelItem);
			uiItem.showTog.isOn = true;
			if (FileTypeUI == 1) { //mp

			}
			if (FileTypeUI == 2) {//kf
			}
			if (FileTypeUI == 3) {//road

			}
			if (FileTypeUI == 4) {//features


			}
			if (FileTypeUI == 5) {//images

			}

		}
	}
		
	public void onReadResultBtn(){
		int re =CppInterface.ReadAllData (FileAddrUI);
		if (re > 0) {
			dealWithResult ();
		}
	}

	public void dealWithResult(){

		if (AlgoTypeUI == 1) {//slam
			updateListUI ();
			List<ChannelInfoRaw> tempList = CppInterface.GetChannelInfo (2);
			//int lastChannelId = tempList [tempList.Count - 1].id;
			if (showFrameUI.isOn) {
				ChannelInfo img = ChannelInfo.getChannel (5, 0);
				ChannelInfo feature = ChannelInfo.getChannel (4, 0);
				if (img!=null && feature!= null ) {
					updateFrameDisplay(img, feature);
				}
			}
//			ChannelInfo item1 = ChannelInfo.getChannel (1, 0);
//			if (item1!=null) {
//				item1.showTog.isOn = true;
//			}
			ChannelInfo item2 = ChannelInfo.getChannel (2, 0);
			if (item2!=null) {
				item2.showTog.isOn = true;
			}
		}
		if (AlgoTypeUI == 2) {//gpe
			updateListUI ();
			ChannelInfo item1 = ChannelInfo.getChannel (3, 0);
			if (item1!=null) {
				item1.showTog.isOn = true;
			}
		}
		if (AlgoTypeUI == 3) {//server
			updateListUI ();
			ChannelInfo item1 = ChannelInfo.getChannel (1, 2);
			if (item1!=null) {
				item1.showTog.isOn = true;
			}
			ChannelInfo item2 = ChannelInfo.getChannel (2, 2);
			if (item2!=null) {
				item2.showTog.isOn = true;
			}
		}
		if (AlgoTypeUI == 4) {//TSL
			updateListUI ();
			ChannelInfo item1 = ChannelInfo.getChannel (6, 0);
			if (item1!=null) {
				item1.showTog.isOn = true;
			}
		}
	}

	public void onSaveListBtn(){
		CppInterface.SaveList (FileTypeUI, Int32.Parse(curChennel.text), FileAddrUI);
	}

	public void onSaveResultBtn(){
		CppInterface.SaveResult (FileAddrUI);
	}

	public void onClearSceneBtn(){
		for (int i = 0; i < listUI.transform.childCount; i++) {
			Transform child = listUI.transform.GetChild (i);
			Destroy (child.gameObject);
		}
		listUI.transform.DetachChildren ();
	}

	public void onSetFileAddr(string str){
		FileAddrUI = str;
	}

	public void onSelAlgo(int algoId){
		AlgoTypeUI = algoId+1;
	}

	public void onSelDataType(int dataTypeId){
		FileTypeUI = dataTypeId+1;
	}

	public void onSetParamBtn(){
		GameObject paramUI = Instantiate(commonObj.ParamRunPrefab) as GameObject;
		paramUI.transform.SetParent (transform,false);
	}

	public void onOpenFile (int actionType) {
		nextActionId = actionType;
		string initPath = "/";
		if (lastOpenHistory.ContainsKey (openFileRecordName)) {
			initPath = lastOpenHistory[openFileRecordName];
		}
		if (nextActionId <= 2) {
			commonObj.GetCommonFileBrowser().OpenFile (initPath, Output);
		} else {
			commonObj.GetCommonFileBrowser().SaveFile (initPath, Output,extensions);
		}

	}

	public void onPlayList(){
		commonObj.mAnimationCont.startPlay ();
	}

		
	private void Output (string[] output) {
		foreach (string path in output) {
			lastOpenHistory [openFileRecordName] = util.GetAviableDir(path);
			SaveHistory ();
			FileAddrUI = path;
			if (nextActionId == 1) {
				onReadFileBtn ();
			}
			if (nextActionId == 2) {
				onReadResultBtn ();
			}
			if (nextActionId == 3) {
				onSaveListBtn ();
			}
			if (nextActionId == 4) {
				onSaveResultBtn ();
			}
		}
	}
}
