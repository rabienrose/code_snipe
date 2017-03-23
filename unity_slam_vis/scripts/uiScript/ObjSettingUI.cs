using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System;

public class ObjSettingUI : MonoBehaviour {

	Transform root;
	InputField L_C_R;
	InputField L_C_G;
	InputField L_C_B;
	InputField H_C_R;
	InputField H_C_G;
	InputField H_C_B;
	InputField C_C_R;
	InputField C_C_G;
	InputField C_C_B;
	InputField con_min;
	InputField con_max;
	InputField H_Min;
	InputField H_Max;
	InputField scale_val;
	Toggle LineModeTog;
	Toggle ConnectTog;
	Toggle ScaleTog;
	Toggle HighlightTog;
	Dropdown filterList;
	Dropdown con_type;
	Button con_sub;
	Button con_add;
	Button scale_sub;
	Button scale_add;

	ChannelInfo item;
	int curId;

	int changeScale = 10;

	void Start () {
		root = GameObject.Find ("ObjSetting").GetComponent<Transform>();
		LineModeTog = root.FindChild ("LineModeTog").GetComponent<Toggle>();
		ConnectTog = root.FindChild ("Connect").GetComponent<Toggle>();
		ScaleTog = root.FindChild ("Scale").GetComponent<Toggle>();
		HighlightTog = root.FindChild ("HighlightTog").GetComponent<Toggle>();
		L_C_R = root.FindChild ("L_C_R").GetComponent<InputField>();
		L_C_G = root.FindChild ("L_C_G").GetComponent<InputField>();
		L_C_B = root.FindChild ("L_C_B").GetComponent<InputField>();
		H_C_R = root.FindChild ("H_C_R").GetComponent<InputField>();
		H_C_G = root.FindChild ("H_C_G").GetComponent<InputField>();
		H_C_B = root.FindChild ("H_C_B").GetComponent<InputField>();
		C_C_R = root.FindChild ("C_C_R").GetComponent<InputField>();
		C_C_G = root.FindChild ("C_C_G").GetComponent<InputField>();
		C_C_B = root.FindChild ("C_C_B").GetComponent<InputField>();
		con_min = root.FindChild ("con_min").GetComponent<InputField>();
		con_max = root.FindChild ("con_max").GetComponent<InputField>();
		H_Min = root.FindChild ("H_Min").GetComponent<InputField>();
		H_Max = root.FindChild ("H_Max").GetComponent<InputField>();
		scale_val = root.FindChild ("scale_val").GetComponent<InputField>();
		filterList = root.FindChild ("FilterDrop").GetComponent<Dropdown>();
		con_type = root.FindChild ("con_type").GetComponent<Dropdown>();
		root.gameObject.SetActive (false);
	}

	public void Show(bool isTure,ChannelInfo _item){
		item = _item;
		if (isTure) {
			L_C_R.text = item.lineColor.r.ToString ();
			L_C_G.text = item.lineColor.g.ToString ();
			L_C_B.text = item.lineColor.b.ToString ();
			H_C_R.text = item.hightColor.r.ToString ();
			H_C_G.text = item.hightColor.g.ToString ();
			H_C_B.text = item.hightColor.b.ToString ();
			C_C_R.text = item.connectColor.r.ToString ();
			C_C_G.text = item.connectColor.g.ToString ();
			C_C_B.text = item.connectColor.b.ToString ();
			LineModeTog.isOn = item.lineModeTog;
			ConnectTog.isOn = item.connectLineTog;
			ScaleTog.isOn = item.scaleTog;
			HighlightTog.isOn = item.hightlightTog;
			con_min.text = item.connectMin.ToString ();
			con_max.text = item.connectMax.ToString ();
			H_Min.text = item.highlightMin.ToString ();
			H_Max.text = item.highlightMax.ToString ();
			scale_val.text = item.scaleVal.ToString ();
			filterList.value = item.filterType;
			con_type.value = item.con_type;
			gameObject.SetActive (true);
		} else {
			gameObject.SetActive (false);
		}

	}

	public void onApplyBtn(){
		item.lineColor.r = Byte.Parse (L_C_R.text);
		item.lineColor.g = Byte.Parse (L_C_G.text);
		item.lineColor.b = Byte.Parse (L_C_B.text);
		item.hightColor.r = Byte.Parse (H_C_R.text);
		item.hightColor.g = Byte.Parse (H_C_G.text);
		item.hightColor.b = Byte.Parse (H_C_B.text);
		item.connectColor.r = Byte.Parse (C_C_R.text);
		item.connectColor.g = Byte.Parse (C_C_G.text);
		item.connectColor.b = Byte.Parse (C_C_B.text);
		item.lineModeTog = LineModeTog.isOn;
		item.connectLineTog = ConnectTog.isOn;
		item.scaleTog = ScaleTog.isOn;
		item.hightlightTog = HighlightTog.isOn;
		item.connectMin = Int32.Parse(con_min.text);
		item.connectMax= Int32.Parse(con_max.text);
		item.highlightMin= Int32.Parse(H_Min.text);
		item.highlightMax= Int32.Parse(H_Max.text);
		item.scaleVal = Int32.Parse (scale_val.text);
		item.filterType = filterList.value;
		item.con_type = con_type.value;
		item.onAdjustParam ();

	}

	public void onScaleAddBtn(){
		int val = Int32.Parse (scale_val.text);
		val = val + changeScale;
		scale_val.text = val.ToString ();
	}

	public void onScaleSubBtn(){
		int val = Int32.Parse (scale_val.text);
		val = val - changeScale;
		scale_val.text = val.ToString ();
	}

	void Update () {
	
	}
}
