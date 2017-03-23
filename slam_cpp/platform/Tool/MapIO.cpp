
#include <string>
#include <mat.h>
#include "Platform.hpp"
#include "ReadGlobalMapFromM.hpp"
#include "SaveGlobalMapToM.hpp"
#include "Frame.hpp"
#include "GlobalMap.hpp"
#include "TypeDef.hpp"

namespace ygomi {
    
    
    void Platform::getGlobalMapFromMatlab(const mxArray* globalMap)
    {
        readGlobalMapFromM(globalMap, reinterpret_cast<long>(m_pGlobalMap.get()));
    }
    
    mxArray* Platform::setGobalMapToMatlab()
    {
        mxArray* globalMap = nullptr;
        saveGlobalMapToM(&globalMap, reinterpret_cast<long>(m_pGlobalMap.get()));
        
        return globalMap;
    }
    
    void Platform::getGobalMapFromMatlab(const std::string& filepath){
        MATFile* pmat = matOpen(filepath.c_str(), "r");
        mxArray* pData = matGetVariable(pmat, "coreDataMatrix");
        readGlobalMapFromM(pData, reinterpret_cast<long>(m_pGlobalMap.get()));
        int fieldCount= mxGetNumberOfFields(pData);
        for (int i=0; i<fieldCount; i++){
            mxArray *subMx = mxGetFieldByNumber(pData, 0, i);
            mxDestroyArray(subMx);
        }
        if (matClose(pmat) != 0) {
            printf("Error closing file %s\n",filepath.c_str());
            return;
        }
    }
    
    void Platform::setGobalMapToMatlab(const std::string& fileName){
        MATFile* pmat = matOpen(fileName.c_str(), "w");
        mxArray* pData;
        saveGlobalMapToM(&pData, reinterpret_cast<long>(m_pGlobalMap.get()));
        int status;
        status = matPutVariable(pmat, "coreDataMatrix", pData);
        if (status != 0) {
            printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
            return;
        }
        int fieldCount= mxGetNumberOfFields(pData);
        for (int i=0; i<fieldCount; i++){
            mxArray *subMx = mxGetFieldByNumber(pData, 0, i);
            mxDestroyArray(subMx);
        }
        if (matClose(pmat) != 0) {
            printf("Error closing file %s\n",fileName.c_str());
            return;
        }
    }
}
