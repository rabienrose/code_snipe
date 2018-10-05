import math
import numpy as np

def get_visible_mp(pose, mps, fx, fy, cx, cy, width, height):
    mps_filter =[]
    for i in range(0, len(mps)):
        mp_norm = np.matrix(np.ones((4,1),np.float))
        mp_norm[0:3,0] = mps[i][0:3,0]
        pose_norm = np.matrix(np.identity(4,np.float))
        pose_norm[0:3,0:4] = pose[0:3,0:4]
        k=np.matrix(np.identity(4,np.float))
        k[0,0] = fx
        k[1,1] = fy
        k[0,2] = cx
        k[1,2] = cy
        posi_p = k*pose_norm*mp_norm
        u=posi_p[0,0]/posi_p[2,0]
        v=posi_p[1,0]/posi_p[2,0]
        if u<width-1 and u>1 and v<height-1 and v>1:
            mps_filter.append(i)
    return mps_filter
