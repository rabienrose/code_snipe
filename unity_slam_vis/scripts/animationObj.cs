using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;

public class animationObj : MonoBehaviour {

	// Use this for initialization
	public float timeD;
	public float timeDForRecording;
	float timerForRecording;
	float updateCurTime;
	int curStep;
	public bool isPause;
	public bool isRecording;
	bool needOneMoreStep;

	int recordFrameCount;
	void Start () {
		commonObj.mAnimationCont = this;
		recordFrameCount = 0;
		timeD = 0.2f;
		timeDForRecording = 0.1f;
		updateCurTime = 0;
		timerForRecording = 0;
		curStep = -1;
		isPause = false;
		needOneMoreStep = false;
	}

	public void startPlay(){
		curStep = 0;
	}

	public void setPause(bool val){
		isPause = val;
	}

	public void setRecording(bool val){
		isRecording = val;
	}

	public void stepOnce(){
		needOneMoreStep = true;
	}

	// Update is called once per frame
	void Update () {
		timerForRecording = timerForRecording + Time.deltaTime;
		updateCurTime = updateCurTime + Time.deltaTime;

		if (isRecording) {
			if (timerForRecording > timeDForRecording) {
				timerForRecording = 0;
				string fileName = String.Format ("c{0}", recordFrameCount.ToString("D8"));
				Application.CaptureScreenshot ("/Volumes/chamo/dataset/vRecord/"+fileName+".png");
				recordFrameCount++;
			}
		}
			
		if (curStep < 0) {
			return;
		}

		if (isPause) {
			if (needOneMoreStep) {
				needOneMoreStep = false;
			}else{
				return;
			}
		}

		if (updateCurTime > timeD) {
			updateCurTime = 0;
			int algoType = commonObj.mMainUI.AlgoTypeUI;
			if (algoType == 1) {
				ChannelInfo img = ChannelInfo.getChannel (5, curStep);
				ChannelInfo feature = ChannelInfo.getChannel (4, curStep);
				if (img != null && feature != null) {
					commonObj.mMainUI.updateFrameDisplay (img, feature);
				} else {
					if (img == null) {
						curStep = -1;
						return;
					}
				}
				ChannelInfo item;

				item = ChannelInfo.getChannel (2, curStep-1);
				if (item!=null) {
					item.showTog.isOn = false;
					item.DelObj ();
				}
				item = null;
				//item = ChannelInfo.getChannel (1, curStep-1);
				if (item!=null) {
					//item.DelObj ();
					//item.showTog.isOn = false;
				}
				item = null;
				item = ChannelInfo.getChannel (2, curStep);
				if (item != null) {
					item.showTog.isOn = true;
				}
				item = null;
				item = ChannelInfo.getChannel (1, curStep);
				if (item!=null) {
					item.showTog.isOn = true;
				}
			}
			if (algoType == 2) {
				ChannelInfo item = ChannelInfo.getChannel (3, 0);
				if (item!=null) {
					if (curStep == 0) {
						item.showCount = 50;
						item.showTog.isOn = true;
					} else if (curStep <= 100) {
						if (item.obj != null) {
							RoadMesh roadobj = item.obj.GetComponent<RoadMesh> ();
							List<RoadSeg> tmpData = item.RdSegData;
							roadobj.addSegments (tmpData.GetRange (curStep * 20, 22));
						}
					} else {
						item.showCount = -1;
						item.showTog.isOn = false;
						item.DelObj ();
						item.showTog.isOn = true;
						curStep = -1;
					}
				}
			}
			if (algoType == 3) {
				if (curStep == 0) {
					commonObj.pausePlayTog.isOn = true;
					ChannelInfo item = ChannelInfo.getChannel (2, 2);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 2);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 1) {
					ChannelInfo item = ChannelInfo.getChannel (2, 3);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 3);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 2) {
					ChannelInfo item = ChannelInfo.getChannel (2, 3);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 3);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (2, 4);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 4);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 3) {
					ChannelInfo item = ChannelInfo.getChannel (2, 4);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 4);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (2, 2);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 2);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (2, 5);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 5);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 4) {
					ChannelInfo item = ChannelInfo.getChannel (2, 6);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 6);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 5) {
					ChannelInfo item = ChannelInfo.getChannel (2, 6);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 6);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (2, 7);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 7);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
				if (curStep == 6) {
					ChannelInfo item = ChannelInfo.getChannel (2, 7);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 7);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (2, 5);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = ChannelInfo.getChannel (1, 5);
					if (item!=null) {
						item.showTog.isOn = false;
					}
					item = null;
					item = ChannelInfo.getChannel (2, 8);
					if (item!=null) {
						item.showTog.isOn = true;
					}
					item = ChannelInfo.getChannel (1, 8);
					if (item!=null) {
						item.showTog.isOn = true;
					}
				}
			}

			if (algoType == 4) {
				ChannelInfo img = ChannelInfo.getChannel (5, curStep+1);
				if (img != null) {
					commonObj.mMainUI.updateFrameDisplay (img, null);
					ChannelInfo item = ChannelInfo.getChannel (6, curStep);
					if (item!= null) {
						item.showTog.isOn = true;
						commonObj.pausePlayTog.isOn = true;
					}
				} else {
					curStep = -1;
					return;
				}

			}
			curStep = curStep + 1;
		}

	}
}
