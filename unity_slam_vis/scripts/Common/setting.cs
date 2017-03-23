using UnityEngine;
using System.Collections;

public class setting {
	public static int MaxAlgoType = 4;
	public static int MaxDataType = 6;
	public static int MaxChannel = 1024;
	public static int MaxLogLines = 40;
	public static string[] DateTypeName = {"", "MapPoint", "KeyFrame", "RoadSeg", "Feature", "Image", "TraficSign", "ActMP" };
	public static string[] AlgoTypeName = {"", "SLAM", "GPE", "Server", "TSL" };
}
