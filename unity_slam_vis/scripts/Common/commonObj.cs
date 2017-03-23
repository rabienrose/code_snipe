using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using SkywardRay.FileBrowser;
using UnityEngine.EventSystems;

public class commonObj: MonoBehaviour {
	public static void Init(){
		mainEventSys = GameObject.Find ("EventSystem").GetComponent<EventSystem> ();
		menuPrefab = Resources.Load ("prefab/ListMenu") as GameObject;
		ListItemPrefab = Resources.Load ("prefab/listItemUI") as GameObject;
		ParamRunPrefab = Resources.Load ("prefab/paramUI") as GameObject;
		mShader = Resources.Load ("pointShader") as Shader;
		mRoadSufM = Resources.Load ("material/RoadSufM") as Material;
		mActMPM = Resources.Load ("material/actmpM") as Material;
		signPrefab = Resources.Load ("prefab/parking") as GameObject;
		groundTer = GameObject.Find ("Terrain");
		uiRootCanvas = GameObject.Find ("Canvas").transform;
		pausePlayTog = uiRootCanvas.Find ("Panel/PausePlay").GetComponent<Toggle>();
		camControl = GameObject.Find ("Main Camera").GetComponent<CameraMovement> ();
		objSettingUI = GameObject.Find ("ObjSetting").GetComponent<ObjSettingUI> ();
	}
	public static Shader mShader;
	public static Material mRoadSufM;
	public static Material mActMPM;
	public static Transform listRoot;

	public static GameObject prefabFileBrower;
	public static GameObject menuPrefab;
	public static GameObject ListItemPrefab;
	public static GameObject ParamRunPrefab;
	public static GameObject signPrefab;

	public static EventSystem mainEventSys;
	public static SkywardFileBrowser fileBrowser;

	public static uiMain mMainUI;
	public static animationObj mAnimationCont;

	public static Transform goundGird;
	public static GameObject groundTer;

	public static Toggle pausePlayTog;
	public static Transform uiRootCanvas;
	public static CameraMovement camControl;
	public static ObjSettingUI objSettingUI;

	public static SkywardFileBrowser GetCommonFileBrowser(){
		if (fileBrowser != null) {
			GameObject.Destroy (fileBrowser.gameObject);
		}
		GameObject obj = GameObject.Find ("Canvas SkywardFileBrowser");
		if (obj != null) {
			GameObject.Destroy (obj);
		}
		fileBrowser = Object.Instantiate (prefabFileBrower).GetComponent<SkywardFileBrowser> ();
		return fileBrowser;
	}
}
