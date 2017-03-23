using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class mapPoint{
	public int id;
	public Vector3 pos;
	public Color32 color;
	public int filter1, filter2, filter3;
};

public class KeyFrameC{
	public int id;
	public Vector3 pos;
	public Vector3 forward;
	public Vector3 right;
	public Vector3 up;
	public Color32 color;
	public int filter1, filter2, filter3;
	public int[] kfLinks1;
	public int[] kfLinks2;
	public int[] kfLinks3;
	public int[] kfLinksVal1;
	public int[] kfLinksVal2;
	public int[] kfLinksVal3;
	public int[] mpLinks;
};

public struct RoadSeg{
	public int id;
	public Vector3 pos;
	public Vector3 normal;
	public float hWidthR, hWidthL;
};

public struct RoadPoint{
	public int id;
	public float r,g,b;
	public float x,y,z;
};

public struct TraficSign{
	public int id;
	public Vector3 pos;
	public Vector3 forward;
	public int typeId;
};

public struct RoadSegCpp{
	public int id;
	public float x, y, z;
	public float normX, normY, normZ;
	public float hWidthR, hWidthL;
};

public struct KeyFrameCpp{
	public int id;
	public float x,y,z;
	public float r00,r01,r02;
	public float r10,r11,r12;
	public float r20,r21,r22;
	public int r,g,b;
	public int filter1, filter2, filter3;
	public int kfLinksCount1, kfLinksCount2, kfLinksCount3;
	public int mpLinksCount;
};

public struct MapPointCpp{
	public int id;
	public float x,y,z;
	public int r,g,b;
	public int filter1, filter2, filter3;
};

public struct ActMapPointCpp{
	public int id;
	public float x,y,z;
	public int r,g,b;
};

public struct FeaturesCpp{
	public int id;
	public int u;
	public int v;
	public int r;
	public int lastU;
	public int lastV;
	public int red, green, blue;
};


public struct ImageCppC{
	public int id;
	public int h;
	public int w;
};

public struct TSCpp{
	public int id;
	public float x,y,z;
	public float forwardx,forwardy,forwardz;
	public int typeId;
}

public class ChannelInfoRaw{
	public string name;
	public int type;
	public int count;
	public int id;
}

public class ActMapPoint{
	public int id;
	public Vector3 pos;
	public Color32 color;
};

