//PUT_VREP_ROMEO_FULLY_FREE_COPYRIGHT_NOTICE_HERE__AUTOGENERATED

#pragma once

#ifdef _WIN32
    #define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

//#include "v_repExtRomeoInclude.h"
#include "v_repExtRomeoSensors.h"
#include "v_repRomeo.h"

int counter = 0;
v_repRomeo::v_repRomeoRobot *ROMEORobot;


// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

