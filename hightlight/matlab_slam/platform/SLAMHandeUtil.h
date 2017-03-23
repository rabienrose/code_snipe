#pragma once

#include <mex.h>
#include "Platform.hpp"
#include <string>

namespace ygomi{
    class Frame;
    class MapPoint;
    class KeyPoint;
    class GlobalMap;
}

ygomi::Platform* getSLAMHandle(const mxArray* mx)
{
    assert(mx);
    uint64* platformAddr = static_cast<uint64*>(mxGetData(mx));
    
    return reinterpret_cast<ygomi::Platform*>(*platformAddr);
}

void setSLAMHandle(const mxArray* mx, ygomi::Platform* platformPtr)
{
    uint64* ptr = static_cast<uint64*>(mxGetData(mx));
    assert(ptr);
    
    ptr[0] = reinterpret_cast<uint64>(platformPtr);
}

std::string getMexString(const mxArray* mx){
    int strLenght = mxGetN(mx);
    char16_t *ss = mxGetChars(mx);
    std::string tStr;
    for (int n = 0;n<strLenght;n++){
        tStr.push_back(ss[n]);
    }
    return tStr;
}

mxArray* setLong2Double(long value){
    mxArray* mx = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double* ptr1 = static_cast<double*>(mxGetData(mx));
    ptr1[0] = (double)value;
    return mx;
}

mxArray* setPtr2Uint64(void* value){
    mxArray* mx = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    uint64* ptr1 = static_cast<uint64*>(mxGetData(mx));
    ptr1[0] = (uint64)value;
    return mx;
    
}

mxArray* setPtr2Uint64List(std::vector<void*> values){
    mxArray* mx = mxCreateNumericMatrix(values.size(), 1, mxUINT64_CLASS, mxREAL);
    uint64* ptrr0 = static_cast<uint64*>(mxGetData(mx));
    for (int i=0; i<values.size(); i++){
        ptrr0[i] = (uint64)values[i];
    }
    return mx;
    
}

long getLong(const mxArray* mx){
    double* ptr1 = static_cast<double*>(mxGetData(mx));
    return static_cast<long>(*ptr1);
}

ygomi::Frame* getFramePtr(const mxArray* mx){
    uint64* handlerAddr = static_cast<uint64*>(mxGetData(mx));
    return reinterpret_cast<ygomi::Frame*>(*handlerAddr);
}

ygomi::GlobalMap* getGlobalMapPtr(const mxArray* mx){
    uint64* handlerAddr = static_cast<uint64*>(mxGetData(mx));
    return reinterpret_cast<ygomi::GlobalMap*>(*handlerAddr);
}

ygomi::MapPoint* getMapPointPtr(const mxArray* mx){
    uint64* handlerAddr = static_cast<uint64*>(mxGetData(mx));
    return reinterpret_cast<ygomi::MapPoint*>(*handlerAddr);
}

ygomi::KeyPoint* getKeyPointPtr(const mxArray* mx){
    uint64* handlerAddr = static_cast<uint64*>(mxGetData(mx));
    return reinterpret_cast<ygomi::KeyPoint*>(*handlerAddr);
}

