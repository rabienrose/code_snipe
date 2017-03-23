#pragma once

#include <mex.h>
#include "TestCaseParser.hpp"
#include <string>

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

mxArray* setFloat2Double(float value){
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

mxArray* setStringList(std::vector<std::string> values){
    int count = values.size();
    mxArray* nameList = mxCreateCellMatrix(count, 1);
    for (int i=0;i<count;i++){
        mxArray* strCell = mxCreateString(values[i].c_str());
        mxSetCell(nameList, i, strCell);
    }
    return nameList;
}

long getLong(const mxArray* mx){
    double* ptr1 = static_cast<double*>(mxGetData(mx));
    return static_cast<long>(*ptr1);
}

ygomi::TestCaseParser* getTestCasePPtr(const mxArray* mx){
    uint64* handlerAddr = static_cast<uint64*>(mxGetData(mx));
    return reinterpret_cast<ygomi::TestCaseParser*>(*handlerAddr);
}

