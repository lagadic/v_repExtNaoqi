//PUT_VREP_ROMEO_FULLY_FREE_COPYRIGHT_NOTICE_HERE__AUTOGENERATED

// Written by Marco Cognetti


#include "v_repExtRomeo.h"

// Define functions for right/left hipYaw joint
// This is important since Aldebaran robot has only one motor for two joints
// In the simulator, there are two joints. I need their names to remap this information

// Denote the name of the right hipYaw joint
#define RHIPYAWPITCHJOINT "RHipYawPitch"
// Denote the name of the left hipYaw joint
#define LHIPYAWPITCHJOINT "LHipYawPitch"

#ifdef _WIN32
#include <windows.h> //Sleep
#undef max

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#else
# include <sys/time.h>
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
#include <string.h>
#define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)
// Definition of the name of the Vrep functions
#define LUA_SEND_STATE_COMMAND "simExtRomeo_sendState"
#define LUA_INIT_THREADS "simExtRomeo_init_threads"
#define LUA_INIT_COMMAND "simExtRomeo_init"
#define ROMEO_STATE_DIM 8 // as an example, let's say that Romeo's state is described by 8 floats. TODO
// Define for testing functions

// Test function
#define LUA_IMU_TEST "simExtIMU_test"

// SENSOR ROOT NAMES DEFINE
#define IMU_ROOT "IMU"

LIBRARY vrepLib;

///////////////////////////////////////////////////////////
/////////////////// TESTING FUNCTIONS /////////////////////
///////////////////////////////////////////////////////////

// IMU
//void LUA_IMU_TEST_CALLBACK(SLuaCallBack* p){
//    // Here the code for testing IMU
//    // INERTIAL SENSORS
//    v_repRomeoSensors::InertialSensor* inSr;
//    simInt pa = simGetObjectHandle("Cuboid");
//    simInt jj = simGetObjectHandle("Revolute_joint0");
//    inSr = new v_repRomeoSensors::InertialSensor(IMU_ROOT, pa);
//    inertialSensorsVrep.push_back(inSr);
//    inSr->updateSensor();
//    simSetJointTargetVelocity(jj,1.0);
//}

///////////////////////////////////////////////////////////
////////////////// CALLBACK FUNCTIONS /////////////////////
///////////////////////////////////////////////////////////

/// \brief Method that is invoked by the LUA script to send the commands
/// \param[in].p Pointer to the Vrep object that is invoking the LUA function



void LUA_SEND_STATE_COMMAND_CALLBACK(SLuaCallBack* p){
  // the callback function of the new Lua command "simExtRomeo_sendState"
  simLockInterface(1);

  int result=-1; // means error

  // Update the robot
  ROMEORobot->update();

  //std::cout << "argCount: " << p->inputArgCount << std::endl;
  if (p->inputArgCount>0)
  { // Ok, we have at least 1 input argument
    if ( (p->inputArgTypeAndSize[0*2+0]==(sim_lua_arg_float|sim_lua_arg_table)) && (p->inputArgTypeAndSize[0*2+1]>=ROMEO_STATE_DIM) )
    { // Ok, we have a table of at least <Romeo_STATE_DIM> floats for the first argument

      float RomeoJointValues[ROMEO_STATE_DIM];
      for (int i=0;i<ROMEO_STATE_DIM;i++)
        RomeoJointValues[i]=p->inputFloat[i];
      int objectHandleWhereScriptIsAttached=p->objectID; // this could be used as identifier of the Romeo (if we have several Romeos in the same scene)

      result=1; // let's say the function succeeded!
    }
    else
      simSetLastError(LUA_SEND_STATE_COMMAND,"Wrong argument type/size."); // output an error
  }
  else
    simSetLastError(LUA_SEND_STATE_COMMAND,"Not enough arguments."); // output an error

  // Now we prepare the return value (one number):
  p->outputArgCount=1; // 1 return value
  p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer
  p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_int;	// The return value is an int
  p->outputArgTypeAndSize[2*0+1]=1;				// Not used (table size if the return value was a table)
  p->outputInt=(simInt*)simCreateBuffer(1*sizeof(result)); // 1 int return value
  p->outputInt[0]=result; // This is the int value we want to return

  simLockInterface(0);
}

//void sighandler(int sig){
//    //std::cout << "STOP THREAD!!!!!!!!!!" << std::endl;
//    t1->interrupt();
//    t2->interrupt();
//    sleep(1);
//    exit(0);
//}

/// \brief Method that is invoked by the LUA script to initialize the robot
/// \param[in].p Pointer to the Vrep object that is invoking the LUA function
void LUA_INIT_COMMAND_CALLBACK(SLuaCallBack* p){
  // The callback function of the new Lua command "simExtRomeo_init"
  simLockInterface(1);

  // Find the correspondences between Aldebaran libraries and Vrep links/joints
  ROMEORobot->findCorrespondences();

  simLockInterface(0);
}

/// \brief Method that is invoked by the LUA script to start NAOqi and HAL executables
/// \param[in].p Pointer to the Vrep object that is invoking the LUA function
void LUA_INIT_THREADS_CALLBACK(SLuaCallBack* p){
  // The callback function of the new Lua command "simExtRomeo_init"
  simLockInterface(1);

  std::cout << "robotID: " << p->objectID << std::endl;

  // Build the robot and initialize the NAOqi and HAL executables
  ROMEORobot = new v_repRomeo::v_repRomeoRobot(p->objectID);

  simLockInterface(0);
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
/// \brief Method that is invoked by Vrep when it starts
/// \details This method is called by Vrep even if you are not using Romeo robot!
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt){

  // Dynamically load and bind V-REP functions:
  // ******************************************
  // 1. Figure out this plugin's directory:
  char curDirAndFile[1024];
#ifdef _WIN32
  GetModuleFileName(NULL,curDirAndFile,1023);
  PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
  getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
  std::string currentDirAndPath(curDirAndFile);
  // 2. Append the V-REP library's name:
  std::string temp(currentDirAndPath);
#ifdef _WIN32
  temp+="\\v_rep.dll";
#elif defined (__linux)
  temp+="/libv_rep.so";
#elif defined (__APPLE__)
  temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

  // 3. Load the V-REP library:
  vrepLib=loadVrepLibrary(temp.c_str());
  if (vrepLib==NULL){
    //std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'Romeo' plugin.\n";
    return(0); // Means error, V-REP will unload this plugin
  }
  if (getVrepProcAddresses(vrepLib)==0){
    //std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'Romeo' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0); // Means error, V-REP will unload this plugin
  }
  // ******************************************

  // Check the version of V-REP:
  // ******************************************
  int vrepVer;
  simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
  if (vrepVer<30000){ // if V-REP version is smaller than 3.00.00
    //std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'Romeo' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0); // Means error, V-REP will unload this plugin
  }
  // ******************************************

  // Inizialize the robot
  ROMEORobot = NULL;

  simLockInterface(1);

  // Here you could handle various initializations
  // etc.

  // Register 2 new Lua commands:


  // The first command:
  int inArgs1[]={1,sim_lua_arg_float|sim_lua_arg_table}; // we expect 1 table of float values
  simRegisterCustomLuaFunction(LUA_SEND_STATE_COMMAND,strConCat("number result=",LUA_SEND_STATE_COMMAND,"(table RomeoState)"),inArgs1,LUA_SEND_STATE_COMMAND_CALLBACK);
  // The second command:
  int inArgs2[]={0}; // we expect no arguments

  // Register init function
  simRegisterCustomLuaFunction(LUA_INIT_COMMAND,strConCat("number result=",LUA_INIT_COMMAND,"(table RomeoState)"),inArgs2,LUA_INIT_COMMAND_CALLBACK);

  // Register the function that starts NAOqi and HAL as threads
  simRegisterCustomLuaFunction(LUA_INIT_THREADS,strConCat("number result=",LUA_INIT_THREADS,"(table RomeoState)"),inArgs2,LUA_INIT_THREADS_CALLBACK);

  // Register IMU test function
  //    simRegisterCustomLuaFunction(LUA_IMU_TEST,strConCat("number result=",LUA_IMU_TEST,"(table RomeoState)"),inArgs2,LUA_IMU_TEST_CALLBACK);

  simLockInterface(0);
  return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
/// \brief Method that is invoked by Vrep when it is ending
/// \details This method is called by Vrep even if you are not using Romeo robot!
VREP_DLLEXPORT void v_repEnd(){

  // If the Romeo robot has been created, then stop the threads
  if(ROMEORobot!=NULL)
    ROMEORobot->killThreads();

  // Here you could handle various clean-up tasks
  unloadVrepLibrary(vrepLib); // release the library
  exit(0);
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
/// \brief Method that is invoked by Vrep at each step
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
  // Keep following 6 lines at the beginning and unchanged:
  simLockInterface(1);
  static bool refreshDlgFlag=true;
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
  void* retVal=NULL;

  // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
  // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
  // in the V-REP user manual.

  if (message==sim_message_eventcallback_refreshdialogs)
    refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

  if (message==sim_message_eventcallback_menuitemselected)
  { // A custom menu bar entry was selected..
    // here you could make a plugin's main dialog visible/invisible
  }

  if (message==sim_message_eventcallback_instancepass)
  {	// This message is sent each time the scene was rendered (well, shortly after) (very often)
    // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

    int flags=auxiliaryData[0];
    bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
    bool instanceSwitched=((flags&64)!=0);

    if (instanceSwitched)
    {
      // React to an instance switch here!!
    }

    if (sceneContentChanged)
    { // we actualize plugin objects for changes in the scene

      //...

      refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
    }
  }

  if (message==sim_message_eventcallback_mainscriptabouttobecalled)
  { // The main script is about to be run (only called while a simulation is running (and not paused!))

  }

  if (message==sim_message_eventcallback_simulationabouttostart)
  { // Simulation is about to start

  }

  if (message==sim_message_eventcallback_simulationended)
  { // Simulation just ended
  }

  if (message==sim_message_eventcallback_moduleopen)
  { // A script called simOpenModule (by default the main script). Is only called during simulation.
    if ( (customData==NULL)||(_stricmp("Nao",(char*)customData)==0) ) // is the command also meant for this plugin?
    {
      // we arrive here only at the beginning of a simulation
    }
  }

  if (message==sim_message_eventcallback_modulehandle)
  { // A script called simHandleModule (by default the main script). Is only called during simulation.
    if ( (customData==NULL)||(_stricmp("Nao",(char*)customData)==0) ) // is the command also meant for this plugin?
    {
      // we arrive here only while a simulation is running
    }
  }

  if (message==sim_message_eventcallback_moduleclose)
  { // A script called simCloseModule (by default the main script). Is only called during simulation.
    if ( (customData==NULL)||(_stricmp("Nao",(char*)customData)==0) ) // is the command also meant for this plugin?
    {
      // we arrive here only at the end of a simulation
    }
  }

  if (message==sim_message_eventcallback_instanceswitch)
  { // Here the user switched the scene. React to this message in a similar way as you would react to a full
    // scene content change. In this plugin example, we react to an instance switch by reacting to the
    // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
    // (see here above)

  }

  if (message==sim_message_eventcallback_broadcast)
  { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

  }

  if (message==sim_message_eventcallback_scenesave)
  { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

  }

  // You can add many more messages to handle here
  //    simInt simStat = simGetSimulationState();
  //    if(simStat == sim_simulation_stopped){
  //        //std::cout << "STOPPED SIMULATION" << std::endl;
  //        if(jointsVrep.size()>0)
  //            simSetJointPosition(jointsVrep[0],90.0);
  //    }

  if(message == sim_message_eventcallback_simulationended){
    // Kill all the executables
    if(ROMEORobot!=NULL)
      ROMEORobot->killThreads();
  }

  if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
  { // handle refresh of the plugin's dialogs
    // ...
    refreshDlgFlag=false;
  }

  // Kill threads
  //signal(SIGABRT, &_termination_handler);
  //signal(SIGTERM, &_termination_handler);
  //signal(SIGINT, &_termination_handler);

  // Keep following unchanged:
  simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
  simLockInterface(0);
  return(retVal);
}

