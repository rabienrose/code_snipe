/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{
    
    Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
    {
    }
    
    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        mspKeyFrames.insert(pKF);
        if(pKF->mnId>mnMaxKFid)
            mnMaxKFid=pKF->mnId;
    }
    
    void Map::AddMapPoint(MapPoint *pMP)
    {
        mspMapPoints.insert(pMP);
    }
    
    void Map::EraseMapPoint(MapPoint *pMP)
    {
        mspMapPoints.erase(pMP);
    }
    
    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        mspKeyFrames.erase(pKF);
    }
    
    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        mvpReferenceMapPoints = vpMPs;
    }
    
    void Map::InformNewBigChange()
    {
        mnBigChangeIdx++;
    }
    
    int Map::GetLastBigChangeIdx()
    {
        return mnBigChangeIdx;
    }
    
    KeyFrame* Map::GetKFbyId(int id){
        std::set<KeyFrame*>::iterator it;
        for (it = mspKeyFrames.begin(); it != mspKeyFrames.end(); ++it)
        {
            if((*it)->mnFrameId==id){
                return *it;
            }
        }
        return NULL;
    }
    
    vector<KeyFrame*> Map::GetAllKeyFrames()
    {
        return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    }
    
    vector<MapPoint*> Map::GetAllMapPoints()
    {
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }
    
    long unsigned int Map::MapPointsInMap()
    {
        return mspMapPoints.size();
    }
    
    long unsigned int Map::KeyFramesInMap()
    {
        return mspKeyFrames.size();
    }
    
    vector<MapPoint*> Map::GetReferenceMapPoints()
    {
        return mvpReferenceMapPoints;
    }
    
    long unsigned int Map::GetMaxKFid()
    {
        return mnMaxKFid;
    }
    
    void Map::clear()
    {
        for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
            delete *sit;
        
        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
            delete *sit;
        
        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }
    
} //namespace ORB_SLAM
