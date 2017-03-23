using UnityEngine;
using System.Collections;
using SkywardRay.FileBrowser;
using System;
using UnityEngine.UI;
using System.Collections.Generic;

public class paramChoose : MonoBehaviour {

	int curChooseId;
	int paramCount;
	uiMain UIRoot;
	List<Dropdown> dropUIList;
	List<InputField> inputUIList;
	int max_history_count;

	private SkywardFileBrowser fileBrowser;
	private string defaultPath = "/Volumes/chamo";
	string[] nameDropList = {"param1Drop", "param2Drop", "param3Drop", "param4Drop"};
	string[] nameInputList = {"Param1", "Param2", "Param3", "Param4"};
	private string[] extensions = { "chamo"};
	void Start () {
		curChooseId=0;
		paramCount = 4;
		max_history_count = 5;
		GameObject mainUIObj = GameObject.Find ("Canvas") as GameObject;
		UIRoot = mainUIObj.GetComponent<uiMain> ();
		inputUIList = new List<InputField>();
		dropUIList = new List<Dropdown> ();
		for (int i = 0; i < paramCount; i++) {
			
			dropUIList.Add (transform.FindChild (nameDropList[i]).GetComponent<Dropdown> ());
			if (UIRoot.inputBoxHistory.ContainsKey (nameDropList [i])) {
				dropUIList [i].AddOptions (UIRoot.inputBoxHistory [nameDropList[i]]);
			}
			inputUIList.Add(transform.FindChild (nameInputList[i]).GetComponent<InputField> ());
			if (UIRoot.inputBoxHistory.ContainsKey (nameDropList [i])) {
				if(UIRoot.inputBoxHistory [nameDropList[i]].Count > 0) {
					inputUIList [i].text = UIRoot.inputBoxHistory [nameDropList[i]] [0];
				}
			} 
		}
	}

	public void onOkBtn(){
		for (int i = 0; i < paramCount; i++) {
			UIRoot.Params[i] = inputUIList [i].text;
			if (inputUIList [i].text != "") {
				if (!UIRoot.inputBoxHistory.ContainsKey (nameDropList [i])) {
					UIRoot.inputBoxHistory[nameDropList [i]] = new List<string>();
				}
				UIRoot.inputBoxHistory [nameDropList [i]].Insert (0, inputUIList [i].text);

				bool findFirstTime = false;
				for(int j=0; j<UIRoot.inputBoxHistory [nameDropList [i]].Count;j++)
				{
					if (inputUIList [i].text == UIRoot.inputBoxHistory [nameDropList [i]][j]) {
						if (findFirstTime) {//find it second time
							UIRoot.inputBoxHistory [nameDropList [i]].RemoveAt (j);
							break;
						}
						findFirstTime = true;
					}
				}

				if (UIRoot.inputBoxHistory [nameDropList [i]].Count > max_history_count) {
					UIRoot.inputBoxHistory [nameDropList [i]].RemoveAt (UIRoot.inputBoxHistory [nameDropList [i]].Count - 1);
				}
			}
		}
		UIRoot.SaveHistory ();
		UIRoot.onRunBtn ();
		onOkCancel ();
	}

	public void onOkCancel(){
		Destroy (transform.gameObject);
	}

	public void onChangeDrop(int id){
		inputUIList [id].text = dropUIList [id].options [dropUIList [id].value].text;
	}

	// Update is called once per frame
	void Update () {
	
	}

	public void onChooseBtn(int paramId){
		curChooseId = paramId;
		string initPath = "/";
		if (UIRoot.lastOpenHistory.ContainsKey (nameDropList [curChooseId])) {
			initPath = UIRoot.lastOpenHistory[nameDropList [curChooseId]];
		}
		if (paramId == 3) {
			commonObj.GetCommonFileBrowser ().SaveFile (initPath, Output, extensions);
		} else {
			commonObj.GetCommonFileBrowser().OpenFile(initPath, Output);
		}

	}

	private void Output (string[] output) {
		foreach (string path in output) {
			inputUIList [curChooseId].text = path;
			UIRoot.lastOpenHistory [nameDropList [curChooseId]] = util.GetAviableDir(path);
			UIRoot.SaveHistory ();
		}
	}
}
