using UnityEngine;
using System;
using System.Collections;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using OpenCVForUnity;

public class CppInterface{
	
	public static float scale = 1.0f;
	public static List<KeyFrameC> GetKFList(int channel){
		List<KeyFrameC> carKFList = new List<KeyFrameC> ();
		int kfCount = GetListCount(2,channel);
		for (int i = 0; i < kfCount; i++) {
			IntPtr pkfLinks1 = new IntPtr ();
			IntPtr pkfLinks2 = new IntPtr ();
			IntPtr pkfLinks3 = new IntPtr ();
			IntPtr pkfLinksVal1 = new IntPtr ();
			IntPtr pkfLinksVal2 = new IntPtr ();
			IntPtr pkfLinksVal3 = new IntPtr ();
			IntPtr pmpLinks = new IntPtr ();
			KeyFrameCpp item = GetKFListCpp (channel, i, ref pkfLinks1, ref pkfLinks2, ref pkfLinks3, ref pkfLinksVal1, ref pkfLinksVal2, ref pkfLinksVal3, ref pmpLinks);
			if (item.id != -1) {
				KeyFrameC itemKF = new KeyFrameC();
				itemKF.id = item.id;
				itemKF.pos = new Vector3 (item.x, item.y, item.z);
				itemKF.pos = itemKF.pos * scale;
				itemKF.right = new Vector3 (item.r00, item.r10, item.r20);
				itemKF.forward = new Vector3 (item.r02, item.r12, item.r22);
				itemKF.up = new Vector3 (item.r01, item.r11, item.r21);
				itemKF.color = new Color32 ((byte)item.r, (byte)item.g, (byte)item.b, 255);
				itemKF.filter1 = item.filter1;
				itemKF.filter2 = item.filter2;
				itemKF.filter3 = item.filter3;
				if (item.mpLinksCount > 0) {
					itemKF.mpLinks = new int[item.mpLinksCount];
					Marshal.Copy (pmpLinks, itemKF.mpLinks, 0, item.mpLinksCount);
				}
				if (item.kfLinksCount1 > 0) {
					itemKF.kfLinks1 = new int[item.kfLinksCount1];
					Marshal.Copy (pkfLinks1, itemKF.kfLinks1, 0, item.kfLinksCount1);
					itemKF.kfLinksVal1 = new int[item.kfLinksCount1];
					Marshal.Copy (pkfLinksVal1, itemKF.kfLinksVal1, 0, item.kfLinksCount1);
				}
				if (item.kfLinksCount2 > 0) {
					itemKF.kfLinks2 = new int[item.kfLinksCount2];
					Marshal.Copy (pkfLinks2, itemKF.kfLinks2, 0, item.kfLinksCount2);
					itemKF.kfLinksVal2 = new int[item.kfLinksCount2];
					Marshal.Copy (pkfLinksVal2, itemKF.kfLinksVal2, 0, item.kfLinksCount2);
				}
				//Debug.Log ("item.kfLinksCount3:"+item.kfLinksCount3);
				if (item.kfLinksCount3 > 0) {
					itemKF.kfLinks3 = new int[item.kfLinksCount3];
					Marshal.Copy (pkfLinks3, itemKF.kfLinks3, 0, item.kfLinksCount3);
					itemKF.kfLinksVal3 = new int[item.kfLinksCount3];
					Marshal.Copy (pkfLinksVal3, itemKF.kfLinksVal3, 0, item.kfLinksCount3);
				}
				carKFList.Add (itemKF);
			}
		}
		return carKFList;
	}

	public static List<mapPoint> GetMPList(int channel){
		List<mapPoint> mpList = new List<mapPoint> ();
		int mpCount = GetListCount(1,channel);;
		for (int i = 0; i < mpCount; i++) {
			MapPointCpp item = GetMPListCpp (channel, i);
			if (item.id != -1) {
				mapPoint itemMP = new mapPoint();
				itemMP.id = item.id;
				itemMP.pos = new Vector3 (item.x, item.y, item.z);
				itemMP.pos = itemMP.pos * scale;
				itemMP.color = new Color32 ((byte)item.r, (byte)item.g, (byte)item.b, 255);
				itemMP.filter1 = item.filter1;
				itemMP.filter2 = item.filter2;
				itemMP.filter3 = item.filter3;
				mpList.Add (itemMP);
			}
		}
		return mpList;
	}

	public static List<ActMapPoint> GetAMPList(int channel){
		List<ActMapPoint> mpList = new List<ActMapPoint> ();
		int mpCount = GetListCount(7,channel);
		for (int i = 0; i < mpCount; i++) {
			ActMapPointCpp item = GetAMPListCpp (channel, i);
			if (item.id != -1) {
				ActMapPoint itemMP = new ActMapPoint();
				itemMP.id = item.id;
				itemMP.pos = new Vector3 (item.x, item.y, item.z);
				itemMP.pos = itemMP.pos * scale;
				itemMP.color = new Color32 ((byte)item.r, (byte)item.g, (byte)item.b, 255);
				mpList.Add (itemMP);
			}
		}
		return mpList;
	}

	public static List<RoadSeg> GetRSList(int channel){
		List<RoadSeg> rsList = new List<RoadSeg> ();
		int rsCount = GetListCount(3,channel);
		Vector3 avPos = new Vector3 ();
		Debug.Log (rsCount);
		for (int i = 0; i < rsCount; i++) {
			RoadSegCpp item = GetRSListCpp (channel, i);
			//if (item.id != -1) {
				RoadSeg itemRS = new RoadSeg();
				itemRS.id = item.id;
			    itemRS.pos = new Vector3 (item.x*0.1f, item.y*0.1f, item.z*0.1f);
				avPos = avPos + itemRS.pos;
				itemRS.normal = new Vector3 (item.normX, -item.normY, item.normZ);
				itemRS.hWidthL = 0.5f;
				itemRS.hWidthR = 0.5f;
				rsList.Add (itemRS);
			//}
		}
		avPos = avPos / rsCount;
		avPos.y = avPos.y - 1;
		for (int i = 0; i < rsList.Count; i++) {
			RoadSeg itemRS = rsList [i];
			itemRS.pos = itemRS.pos - avPos;
			itemRS.pos = itemRS.pos * scale;
			rsList [i]= itemRS;
		}
		return rsList;
	}

	public static List<FeaturesCpp> GetFPList(int channel){
		List<FeaturesCpp> fpList = new List<FeaturesCpp> ();
		int fpCount = GetListCount(4,channel);
		for (int i = 0; i < fpCount; i++) {
			FeaturesCpp item = GetFPListCpp (channel, i);
			if (item.id != -1) {
				fpList.Add (item);
			}
		};
		return fpList;
	}

	public static List<Mat> GetIMList(int channel){
		List<Mat> imList = new List<Mat> ();
		int imCount = GetListCount(5,channel);
		for (int i = 0; i < imCount; i++) {
			IntPtr pData = new IntPtr ();
			ImageCppC item = GetIMListCpp (channel, i, ref pData);
			if (item.id != -1) {
				Mat imgMat = new Mat (item.h, item.w, CvType.CV_8UC1);
				byte[] data = new byte[item.h * item.w];
				Marshal.Copy (pData, data, 0, item.h * item.w);
				imgMat.put (0, 0, data);
				imList.Add (imgMat);
			}
		}
		return imList;
	}

	public static List<TraficSign> GetTSList(int channel){
		List<TraficSign> tsList = new List<TraficSign> ();
		int tsCount = GetListCount(6,channel);
		for (int i = 0; i < tsCount; i++) {
			TSCpp item = GetTSListCpp (channel, i);
			if (item.id != -1) {
				TraficSign itemTS;
				itemTS.id = item.id;

				itemTS.pos = new Vector3(item.x, item.y, item.z);
				itemTS.pos = itemTS.pos * scale;
				itemTS.forward = new Vector3(item.forwardx, item.forwardy, item.forwardz);
				itemTS.typeId = item.typeId;
				tsList.Add (itemTS);
			}
		};
		return tsList;
	}

	public static List<ChannelInfoRaw> GetAllChannelInfo(){
		List<ChannelInfoRaw> re = new List<ChannelInfoRaw> ();
		for (int i = 1; i <= setting.MaxDataType; i++) {
			re.AddRange(GetChannelInfo (i));
		}
		return re;
	}

	public static List<ChannelInfoRaw> GetChannelInfo(int type){
		List<ChannelInfoRaw> re = new List<ChannelInfoRaw> ();
		for (int i = 0; i < setting.MaxChannel; i++) {
			int count = GetListCount (type, i);
			if (count > 0) {
				ChannelInfoRaw item = new ChannelInfoRaw();
				//string channelName = GetChannelName (type, i);
				item.count = count;
				item.type = type;
				item.id = i;
				item.name = "null";
				re.Add (item);
			}
		}
		return re;
	}

	//const string libAddr = "/Volumes/chamo/working/matlab_slam/v2.0/cplusplus/Visualisation/libViewInterface.dylib";
	//const string libAddr = "/usr/local/lib/libViewInterface.so";
	//const string libAddr = "/Library/AlgoVisPlatform/libViewInterface.dylib";
	const string libAddr = "libViewInterface";
	[DllImport(libAddr)] public static extern int InitAlgo (int algoType);
	[DllImport(libAddr)] public static extern int ReadSingleData(string fileName, int type, int channel);
	[DllImport(libAddr)] public static extern int ReadAllData(string fileName);
	[DllImport(libAddr)] public static extern int SaveList(int type, int channel, string fileNameC);
	[DllImport(libAddr)] public static extern int SaveResult(string fileNameC);
	[DllImport(libAddr)] public static extern void NewViewHandle ();
	[DllImport(libAddr)] public static extern int GetListCount (int type, int channel);
	[DllImport(libAddr)] public static extern KeyFrameCpp GetKFListCpp (int channel, int ind, ref IntPtr pkfLinks1, ref IntPtr pkfLinks2, ref IntPtr pkfLinks3, ref IntPtr pkfLinksVal1, ref IntPtr pkfLinksVal2, ref IntPtr pkfLinksVal3, ref IntPtr pmpLinks);
	[DllImport(libAddr)] public static extern MapPointCpp GetMPListCpp (int channel, int ind);
	[DllImport(libAddr)] public static extern ActMapPointCpp GetAMPListCpp (int channel, int ind);
	[DllImport(libAddr)] public static extern RoadSegCpp GetRSListCpp (int channel, int ind);
	[DllImport(libAddr)] public static extern FeaturesCpp GetFPListCpp (int channel, int ind);
	[DllImport(libAddr)] public static extern ImageCppC GetIMListCpp (int channel, int ind, ref IntPtr pData);
	[DllImport(libAddr)] public static extern TSCpp GetTSListCpp (int channel, int ind);
	[DllImport(libAddr)] public static extern void SetParam(int ind, string val);
	[DllImport(libAddr)] public static extern string GetChannelName(int type, int ChannelInfo);

}

