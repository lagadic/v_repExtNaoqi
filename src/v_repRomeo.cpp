#include "v_repRomeo.h"
#include "boost/bind.hpp"

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alvision/alvisiondefinitions.h>
#include <alcommon/alproxy.h>

/////////////////////////////////////////
/////// Methods of class v_repRomeo ///////
/////////////////////////////////////////
namespace v_repRomeo{
#define RHIPYAWPITCHJOINT "RHipYawPitch"
#define LHIPYAWPITCHJOINT "LHipYawPitch"
#define LSHOULDERPITCHJOINT "LShoulderPitch"
#define PATH_TO_SEARCH "Hand"
#define IMU_ROOT "IMU"
#define UI_NAME "ROMEO_UI"
#define UI_HANDS 4         // GUI handle that enables/disables the hands
#define UI_CAM_ON 7        // GUI handle that enables/disables the cameras
#define UI_CAM_TOP_BOT 8   // GUI handle that selects which camera to use (if camera active)
#define UI_FSR 10          // GUI handle that enables/disables the FSR sensors

/////////////////////////////////////////////////
//////////// Thread functions ///////////////////
/////////////////////////////////////////////////

void v_repRomeoRobot::launchEx(const char* pathToEx){
  system(pathToEx);
}

void v_repRomeoRobot::launchExHAL(std::string pathToEx){
  system(pathToEx.c_str());
}

void v_repRomeoRobot::killNAOqiEx(){
  system("killall -9 naoqi-ex");
}

void v_repRomeoRobot::killHALEx(){
  system("killall -9 hal-ex");
}

void v_repRomeoRobot::killEx(){
  killNAOqiEx();
  killHALEx();
}

void v_repRomeoRobot::initModelInterface(){
  std::cout<<"enter initModelInterface"<<std::endl;
  simSDKHome = getenv ("SIMULATOR_SDK_HOME");
  std::string pathToModel(simSDKHome);
  //pathToModel+="/share/alrobotmodel/models/NAO_H25_V40.xml";
  pathToModel+="/share/alrobotmodel/models/ROMEOH37.xml";
  int ROMEOqiId = 9559;
#ifndef WIN32
  struct sigaction new_action;
  /* Set up the structure to specify the new action. */
  //            new_action.sa_handler = _termination_handler;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
# else
  signal( SIGINT, _termination_handler );
#endif

  try{
    model = new Sim::Model(pathToModel);
    hal   = new Sim::HALInterface(model, ROMEOqiId);
  }catch(const std::exception& e){
    std::cerr << e.what() << std::endl;
    /// @todo: Add a getLine to inform user on Windows
    exit(-1);
  }
}

void v_repRomeoRobot::findVrepTree(int &objHandle, std::vector<std::string> &names, std::vector<simInt> &handleNames){
  // Take the structure of the robot where is attached the Vrep script
  std::vector<int> toExplore;
  toExplore.push_back(objHandle);
  while (toExplore.size()!=0){
    objHandle=toExplore[toExplore.size()-1];
    toExplore.pop_back();
    int index=0;
    int childHandle=simGetObjectChild(objHandle,index++);
    //std::cout << "childHandle: " << childHandle << std::endl;
    if(childHandle!=-1){
      //std::cout << "nameUp: " << simGetObjectName(childHandle) << std::endl;
      names.push_back(simGetObjectName(childHandle));
      handleNames.push_back(childHandle);
    }
    while (childHandle!=-1){
      toExplore.push_back(childHandle);
      childHandle=simGetObjectChild(objHandle,index++);
      if(childHandle!=-1){
        //std::cout << "nameDw: " << simGetObjectName(childHandle) << std::endl;
        names.push_back(simGetObjectName(childHandle));
        handleNames.push_back(childHandle);
      }
    }
  }
  int size = handleNames.size();
  std::cout << "taille = " << size << std::endl;
}

void v_repRomeoRobot::initThreads(){
  // Launch HAL executable
  std::string pathToExHAL(simSDKHome);
  pathToExHAL+="/bin/hal-ex";
  pathToExHAL+=" -s hal-ipc9559 -p HAL/Robot/Type:string=Romeo -p HAL/Simulation:int=1 -p HAL/SimShdId:int=9559";
  //std::cout << "File to be launched: " << pathToExHAL.c_str() << std::endl;

  // Launch NAOqi executable
  std::string pathToEx(simSDKHome);
  pathToEx+="/bin/naoqi-ex";

  ///////////////////////////////////////////////
  ////// Thread option //////////////////////////
  ///////////////////////////////////////////////
  t2 = new boost::thread(boost::bind(&v_repRomeoRobot::launchExHAL,this,pathToExHAL));
  //sleep(2.5);
  t1 = new boost::thread(&v_repRomeoRobot::launchExHAL,this,pathToEx);

  ///////////////////////////////////////////////
  ////// Process option /////////////////////////
  ///////////////////////////////////////////////
  //pidHAL = fork();
  ////std::cout << "Creating process HAL " << pidHAL << std::endl;
  //if (pidHAL == 0){
  //   //std::cout << "HAL loop: " << pidHAL << std::endl;
  //   launchExHAL(pathToExHAL);
  //}else{
  //   //std::cout << "I'm the parent, my child is from HAL " << pidHAL << std::endl;
  //   pidNAOqi = fork();
  //   //std::cout << "Creating process NAOQI " << pidNAOqi << std::endl;
  //   //std::cout << "Recall process HAL " << pidHAL << std::endl;
  //   if (pidNAOqi == 0){
  //      //std::cout << "NAOQI loop: " << pidHAL << std::endl;
  //      launchExHAL(pathToEx);
  //   }else{
  //       //std::cout <<  "I'm the parent of NAOqi, my child is from NAOQI " << pidNAOqi << std::endl;
  //       //std::cout << "Recall process NAOQI from main " << pidNAOqi << std::endl;
  //       //std::cout << "Recall process HAL from main " << pidHAL << std::endl;
  //   }
  // }
}

void v_repRomeoRobot::getVrepStruct(std::vector<std::string>& namesVrep, std::vector<simInt>&handlesVrep, int &objHandle){
  // Try to get the current tree
  findVrepTree(objHandle,namesVrep, handlesVrep);
}

void v_repRomeoRobot::getAldebStruct(){
  // Take the structure of the robot from ROMEO libraries
  angleActuators = model->angleActuators();
  coupledActuators = model->coupledActuators();  ///MOD BENOIT
  //std::cout << "angleActnr: " << angleActuators.size() << std::endl;
  int i = 1;//MODMARCO
  for(std::vector<const Sim::AngleActuator*>::const_iterator it = angleActuators.begin() ; it != angleActuators.end() ;++ it){
    std::cout<<i<<". setActuatorValue YO: " << (*it)->name() << ": " << (*it)->startValue()*180/M_PI << std::endl;
    float actuatorPosition = hal->fetchAngleActuatorValue(*it);
    // Get the current structure
    const Sim::AngleSensor* angleSensor = model->angleSensor((*it)->name());
    hal->sendAngleSensorValue(angleSensor, actuatorPosition);
    //std::cout << "setSensorValue : " << angleSensor->name() << " to " << actuatorPosition << std::endl;
    i++; //MODMARCO
  }

  //// MODBENOIT///
  i=1;
  for(std::vector<const Sim::CoupledActuator*>::const_iterator it = coupledActuators.begin() ; it != coupledActuators.end() ;++ it){
    std::cout<<i<<". setcoupledActuatorValue YO: " << (*it)->name() << ": " << (*it)->startValue()*180/M_PI << std::endl;
    float coupledActPosition = hal->fetchCoupledActuatorValue(*it);
    // Get the current structure
    const Sim::CoupledSensor* coupledSensor = model->coupledSensor((*it)->name());
    hal->sendCoupledSensorValue(coupledSensor, coupledActPosition);
    std::cout << "setSensorValue : " << coupledSensor->name() << " to " << coupledActPosition << std::endl;
    i++;
  }
  //// MODBENOIT
}

void v_repRomeoRobot::findAldebVrepCorr(std::vector<std::string>&namesVrep, std::vector<simInt>&handlesVrep){
  int i;
  int j = 0, k = -1; //MODMARCO
  bool foundCorr = false;
  int size = namesVrep.size();
  std::vector<const Sim::AngleActuator*>::const_iterator it;

  // Find the corrispondences
  std::cout << "Finding corrispondences" << std::endl;
  for(it = angleActuators.begin() ; it != angleActuators.end() ;++ it){
    foundCorr = false;
    std::cout<<"AngleActuator n° "<<j<<": "<<(*it)->name()<<std::endl;//MODMARCO
    j++;//MODMARCO
    for(i=0;i<size;++i){
      if (std::string::npos != namesVrep[i].find((*it)->name())){
        k++;
        jointsVrep.push_back(handlesVrep[i]);
        foundCorr = true;
        /**************************----MODMARCO----***********************************/
        std::cout<<"JointsVrep n° "<<k<<": "<<jointsVrep[k]<<std::endl<<std::endl;
        if((*it)->name() == LSHOULDERPITCHJOINT){
          std::cout<<"PROVA LSHOULDERPITCHJOINT"<<std::endl;
          std::cout<<"handlesVrep[i] :"<<handlesVrep[i]<<std::endl;
          lShoulderPitchJoint = handlesVrep[i];
        }
        /**************************----MODMARCO----***********************************/
        if((*it)->name() == RHIPYAWPITCHJOINT){
          rHipYawPitchJoint = handlesVrep[i];
        }else if((*it)->name() == LHIPYAWPITCHJOINT){
          lHipYawPitchJoint = handlesVrep[i];
          lastValidlHipYawPitchCommand = (*it)->startValue();
        }
        break;
      }
    }
    // To have 1 VS 1 corrispondence with Romeo list
    if(!foundCorr){
      jointsVrep.push_back(-1);
      //std::cout << "WARNING: " << (*it)->name() << " not found!" << std::endl;
    }
  }

  ////MOD BENOIT
  std::vector<const Sim::CoupledActuator*>::const_iterator itt;
  // Find the corrispondences for the hands
  std::cout << "Finding corrispondences for the hands" << std::endl;
  for(itt = coupledActuators.begin() ; itt != coupledActuators.end() ;++ itt){
    foundCorr = false;
    std::cout<<"CoupledActuator n° "<<j<<": "<<(*itt)->name()<<std::endl;//MODMARCO
    j++;//MODMARCO
    for(i=0;i<size;++i){
      if (std::string::npos != namesVrep[i].find((*itt)->name())){
        k++;
        jointsVrep.push_back(handlesVrep[i]);
        foundCorr = true;
        std::cout<<"JointsCoupledVrep n° "<<k<<": "<<jointsVrep[k]<<std::endl<<std::endl;
        break;
      }
    }
  }
  ////MOD BENOIT ////

  //EDIT BY ME
  //it = angleActuators.begin();
  std::cout<<"STAMPA MIAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl<<"vector jointsVrep:"<<std::endl;
  for (i=0;i<jointsVrep.size();++i){
    std::cout<<"Joint n. "<<i<<": "<<jointsVrep[i]<<std::endl;
  }
  /*std::cout<<"vector handlesVrep:"<<std::endl;
    for (i=0;i<handlesVrep.size();++i){
        std::cout<<"Handle n. "<<i<<": "<<handlesVrep[i]<<std::endl;
    }*/
}

void v_repRomeoRobot::initVariables(){
  // Inizialization of joints
  rHipYawPitchJoint            = -1;
  lastValidlHipYawPitchCommand = 0.0;

  // Inizialization of hands
  v_repRomeoLeftHand             = NULL;
  v_repRomeoRightHand            = NULL;

  // Initialization of threads
  t1 = NULL;
  t2 = NULL;
}

void v_repRomeoRobot::initSensors(){
  //initInertialSensors();
  initCameraSensors();
  //initFSRSensors();
}

void v_repRomeoRobot::initInertialSensors(){
  // INERTIAL SENSORS
  std::vector<const Sim::InertialSensor*> inertialSensors = model->inertialSensors();
  int size = inertialSensors.size();
  v_repRomeoSensors::InertialSensor* inSr;
  for(int i=0;i<size;++i){
    inSr = new v_repRomeoSensors::InertialSensor(IMU_ROOT, vrepId, inertialSensors[i]);
    if(inSr->sensorHandle != -1 && inSr->getMassHandle()!=-1 && inSr->getForceHandle()!=-1)
      inertialSensorsVrep.push_back(inSr);
    std::cout << "InertialSensor: " << inertialSensors[i]->name() << " handle: " << inSr->sensorHandle << std::endl;
  }
}

void v_repRomeoRobot::initCameraSensors(){
  // Here I have to set a sleep since the ALVideoDeviceProxy needs NAOqi
  // running. Without it, it tries to connect while NAOqi is still not running
  //        sleep(2);

  // Get the GUI
  simInt out = simGetUIButtonProperty(UIHandle,UI_CAM_ON);
  simInt out2, bit;
  camerasStatus = 2;

  // With two separate cameras
  cameraLeft = NULL;
  cameraRight = NULL;
  cameraLeftEye = NULL;
  cameraRightEye  = NULL;

  // If the 8-th bin is set to 0, then activate the camera
  out = isNthBitSet(out,8);
  std::cout << "CAMERA out = " << out << std::endl;  //MODBENOIT
  //  if(out==0){    ///MOD BENOIT

  //    // Get the button that selects which camera is running
  //    out2 = simGetUIButtonProperty(UIHandle,UI_CAM_TOP_BOT);
  //    // If the button is up (0) the top camera has to be activated
  //    out2 = isNthBitSet(out2,8);
  //    if(out2==1)
  //      bit = 1;
  //    else
  //      bit = 0;

  // Init the camera(s)
  // CAMERA SENSORS
  std::vector<const Sim::CameraSensor*> cameraSensors = model->cameraSensors();

  int size = cameraSensors.size();
  std::cout << "CameraSize: " << cameraSensors.size() << std::endl;
  v_repRomeoSensors::CameraSensor* camSr;

  //MOD BENOIT      camerasStatus     = computeCameraStatus(out,out2);
  /*MOD BENOIT*/camerasStatus = 0;
  cameraNAOqiStatus = cameraProxy->getActiveCamera();

  std::cout << "******CameraStatus: " << camerasStatus << std::endl;
  std::cout << "******NAOqiStatus: " << cameraNAOqiStatus << std::endl;

  // i TO BE INITIALIZED TO 0!
  for(int i=0;i<size;++i){
    //                if(i==0 && bit == 1 || i==1 && bit == 0){
    camSr = new v_repRomeoSensors::CameraSensor(cameraSensors[i]->name(), vrepId, cameraSensors[i]);
    std::cout << "cameraSensorHandle: " << camSr->sensorHandle << std::endl;

    if(camSr->sensorHandle != -1){
      // With vector
      //                        cameraSensorsVrep.push_back(camSr);
      // With four separate cameras
      camSr->sendResolution(cameraProxy);
      if(i == 0){
        cameraLeft = camSr;
      }else if(i == 1){
        cameraLeftEye = camSr;
      }else if(i == 2){
        cameraRight = camSr;
      }else if(i == 3){
        cameraRightEye = camSr;
      }
    }
    //                }
  }
  //            std::cout << "cameraStatus " << camerasStatus << std::endl;
  /* }else{    MOD BENOIT
    camerasStatus     = 2;
    cameraNAOqiStatus = cameraProxy->getActiveCamera();
  } */

  // Initialize the camera from NAOqi side
  if(camerasStatus == 0){
    cameraProxy->setActiveCamera(1);
    cameraLeftEye->activeCamera();
    cameraLeftEye->sendResolution(cameraProxy);
    cameraLeft->disableCamera();
    cameraRight->disableCamera();
    cameraRightEye->disableCamera();
  }else if(camerasStatus == 1){
    cameraProxy->setActiveCamera(0);
    cameraBottom->activeCamera();
    cameraBottom->sendResolution(cameraProxy);
    cameraTop->disableCamera();
  }
  cameraNAOqiStatus = cameraProxy->getActiveCamera(); //MOD Benoit

  std::cout << "InitCameraStatus: " << camerasStatus << std::endl;
  std::cout << "InitNAOqiStatus: " << cameraNAOqiStatus << std::endl;
}


void v_repRomeoRobot::initFSRSensors(){
  simInt out = simGetUIButtonProperty(UIHandle,UI_FSR);
  out = isNthBitSet(out,8);
  if(out == 0){
    // FSR SENSORS: I activate the sensors only if it is needed
    simInt leftAnkleRollHandle  = jointsVrep[17];
    simInt rightAnkleRollHandle = jointsVrep[23];

    std::vector<const Sim::FSRSensor*> FSRSensors = model->fsrSensors();
    int size = FSRSensors.size();

    v_repRomeoSensors::FSRSensor* FSRSr;

    // HP: It is based on the list given by the method: FSRSensors[i]->name(). First all left than all right sensors
    for(int i=0;i<size;++i){
      std::cout << "FSRSensor[" << i << "]: " << FSRSensors[i]->name() << std::endl;
      if(i<4){
        // Left foot
        std::cout << "FSRSensorRootLeft[" << i << "]: " << simGetObjectName(leftAnkleRollHandle) << std::endl;
        FSRSr = new v_repRomeoSensors::FSRSensor(FSRSensors[i]->name(), leftAnkleRollHandle, FSRSensors[i]);
      }else{
        // Right foot
        std::cout << "FSRSensorRootRight[" << i << "]: " << simGetObjectName(rightAnkleRollHandle) << std::endl;
        FSRSr = new v_repRomeoSensors::FSRSensor(FSRSensors[i]->name(), rightAnkleRollHandle, FSRSensors[i]);
      }
      FSRSensorsVrep.push_back(FSRSr);
    }
  }
}

void v_repRomeoRobot::initHands(){
  simInt out = simGetUIButtonProperty(UIHandle,UI_HANDS);

  // If the 8-th bin is set to 0, then activate the hands
  //if(isNthBitSet(out,8)==0){
  // HANDS
  int leftWristHandle  = jointsVrep[10];
  int rightWristHandle = jointsVrep[17];
  std::vector<const Sim::CoupledActuator*> ca = model->coupledActuators();

  int i=0;
  // Create hand structure
  int size = ca.size();
  for(std::vector<const Sim::CoupledActuator*>::const_iterator it = ca.begin(); it != ca.end(); ++it){
    // Left hand
    if(i==0){
      v_repRomeoLeftHand = new v_repRomeo::v_repRomeoHand(ca[i], model->coupledSensor(ca[i]->name()),leftWristHandle);
      v_repRomeoLeftHand->activeHand();
    }else{
      v_repRomeoRightHand = new v_repRomeo::v_repRomeoHand(ca[i], model->coupledSensor(ca[i]->name()),rightWristHandle);
      v_repRomeoRightHand->activeHand();
    }
    i++;
  }
  //}
}

void v_repRomeoRobot::initProxy(){
  // Here the sleep is needed since it NAOqi has to load first the ALVideoDevice module
  sleep(2);
  cameraProxy = new AL::ALVideoDeviceProxy("127.0.0.1");
  // If I set this I have problems with the resolution
  // cameraProxyName = cameraProxy->subscribe("camera",AL::kQQVGA,AL::kRgbColorSpace,30);
  //std::cout << "cameraProxyName: " << cameraProxyName.c_str() << std::endl;
}

void v_repRomeoRobot::initPose(){
  std::cout<<"ENTER INITPOSE"<<std::endl;
  // Set initial values for the joints
  int angleActSize = angleActuators.size();
  int h=0;
  std::cout<<"angleActSize: "<<angleActSize<<std::endl;
  for(int i = 0;i<angleActSize;++i){
    if(jointsVrep[i]!=-1 && !isnan(angleActuators[i]->startValue())){
      if(jointsVrep[i] == lHipYawPitchJoint){
        lastValidlHipYawPitchCommand = angleActuators[i]->startValue();
      }
      simSetJointTargetPosition(jointsVrep[i],angleActuators[i]->startValue()); //MODMARCO
      std::cout << "setting: " << simGetObjectName(jointsVrep[i]) << " to: " << angleActuators[i]->startValue()*180/M_PI << std::endl;
      const Sim::AngleSensor* angleSensor = model->angleSensor(angleActuators[i]->name());
      hal->sendAngleSensorValue(angleSensor, angleActuators[i]->startValue());
    }else if(jointsVrep[i] == rHipYawPitchJoint && isnan(angleActuators[i]->startValue())){
      simSetJointTargetPosition(jointsVrep[i],lastValidlHipYawPitchCommand);
      std::cout << "setting: " << simGetObjectName(jointsVrep[i]) << " to: " << lastValidlHipYawPitchCommand*180/M_PI << std::endl;
      const Sim::AngleSensor* angleSensor = model->angleSensor(angleActuators[i]->name());
      hal->sendAngleSensorValue(angleSensor, lastValidlHipYawPitchCommand);
    }
    ++h;
  }

  ////MOD BENOIT
  // Set initial values for the coupledActuators
  int coupledActSize = coupledActuators.size();
  std::cout<<"CoupledActSize: "<<coupledActSize<< " and h =  " << h<<std::endl;
  for(int i = 0;i<coupledActSize;++i){
    if(jointsVrep[h]!=-1 && !isnan(coupledActuators[i]->startValue())){
      simSetJointTargetPosition(jointsVrep[h],coupledActuators[i]->startValue()); //MODMARCO
      std::cout << "setting: " << simGetObjectName(jointsVrep[h]) << " to: " << coupledActuators[i]->startValue()*180/M_PI << std::endl;
      const Sim::CoupledSensor* coupledSensor = model->coupledSensor(coupledActuators[i]->name());
      hal->sendCoupledSensorValue(coupledSensor, coupledActuators[i]->startValue());
    }
    h++;
  }
  ////MOD BENOIT

}



int v_repRomeoRobot::isNthBitSet (int &c, int n) {
  return ((c & (int)pow(2,n)) != 0);
}

int v_repRomeoRobot::computeCameraStatus(int &a, int &b) const{
  if(a == 1)
    return 2;
  else if (a == 0 && b == 0)
    return 0;
  else
    return 1;
}

void v_repRomeoRobot::update(){
  updatePose();
  updateSensors();
  //updateHands();
}

void v_repRomeoRobot::updateSensors(){
  //updateInertialSensors();
  updateCameraSensors();
  //updateFSRSensors();
}

void v_repRomeoRobot::updatePose(){
  int i, actSize = angleActuators.size();
  float jointPos, actuatorPosition;
  for(i = 0;i<actSize;++i){
    if(jointsVrep[i]!=-1){
      simGetJointPosition(jointsVrep[i],&jointPos);
      actuatorPosition = hal->fetchAngleActuatorValue(angleActuators[i]);
      //std::cout << "actPos: " << actuatorPosition*180/M_PI << " for: " << simGetObjectName(jointsVrep[i])  << std::endl;
      if(!isnan(actuatorPosition) && jointPos != actuatorPosition){
        /// This test will fail if actuatorPosition is a NaN.
        if(actuatorPosition != actuatorPosition){
          actuatorPosition = angleActuators[i]->startValue();
        }
        const Sim::AngleSensor* angleSensor = model->angleSensor(angleActuators[i]->name());
        hal->sendAngleSensorValue(angleSensor, actuatorPosition);

        // Set the joint value in Vrep
        simSetJointTargetPosition(jointsVrep[i],actuatorPosition);
        if(jointsVrep[i] == lHipYawPitchJoint){
          lastValidlHipYawPitchCommand = actuatorPosition;
          ////std::cout << "Commands for lhipyawpitch: " << lastValidlHipYawPitchCommand << std::endl;
        }
      }else if(jointsVrep[i] == rHipYawPitchJoint){

        const Sim::AngleSensor* angleSensor = model->angleSensor(angleActuators[i]->name());
        hal->sendAngleSensorValue(angleSensor, lastValidlHipYawPitchCommand);

        // Set the joint value in Vrep
        simSetJointTargetPosition(jointsVrep[i],lastValidlHipYawPitchCommand);
        //std::cout << "Commands for rhipyawpitch: " << lastValidlHipYawPitchCommand << std::endl;
      }
    }
  }
}


void v_repRomeoRobot::updateInertialSensors(){
  // Update inertial sensors
  std::vector<v_repRomeoSensors::InertialSensor*>::iterator IMUit;
  for(IMUit=inertialSensorsVrep.begin();IMUit!=inertialSensorsVrep.end();++IMUit){
    std::cout << "NameIMU sensor: " << (*IMUit)->sensorName.c_str() << std::endl;
    (*IMUit)->updateSensor(hal);
  }
}

void v_repRomeoRobot::updateCameraSensors(){

  /* MOD BENOIT  simInt out = simGetUIButtonProperty(UIHandle,UI_CAM_ON);
  out = isNthBitSet(out,8);

  // Get the button that selects which camera is running
  simInt out2 = simGetUIButtonProperty(UIHandle,UI_CAM_TOP_BOT);
  simInt out3 = out2;
  out2 = isNthBitSet(out2,8);

  // Compute the status based on the interface buttons
  out = computeCameraStatus(out,out2);
*/
  // Check which camera is selected
  simInt out2 = cameraProxy->getActiveCamera();
  simInt out=0, out3;

  std::cout << "cameraStatusBefore: " << camerasStatus << " intStatus: " << out << " NAOqiStatus: " << out2 << " Camera Proxy = " << cameraNAOqiStatus << std::endl;
  //        std::cout << "cameraResolution: " << cameraProxy->getResolution(cameraProxyName) << std::endl;

  if(camerasStatus == out && cameraNAOqiStatus == out2){

    // If the new status is equal to the old one, just perform the usual step
    // Update camera sensors

    // With vector
    //            std::vector<v_repNaoSensors::CameraSensor*>::iterator Camit;
    //            for(Camit=cameraSensorsVrep.begin();Camit!=cameraSensorsVrep.end();++Camit){
    //                std::cout << "NameCamera sensor: " << (*Camit)->sensorName.c_str() << std::endl;
    //                (*Camit)->updateSensor(hal);
    //            }

    std::cout << "cameraLeftEye->sensorEnable: " << cameraLeftEye->sensorEnable << std::endl;
    std::cout << "cameraRightEye->sensorEnable: " << cameraRightEye->sensorEnable << std::endl;

    // With two separate cameras
    if(cameraLeftEye->sensorEnable){
      std::cout << "sending data continuous: " << cameraNAOqiStatus << std::endl;
      cameraLeftEye->updateSensor(hal);
    }else if(cameraRightEye->sensorEnable){
      cameraRightEye->updateSensor(hal);
    }else if(cameraRight->sensorEnable){
      cameraRight->updateSensor(hal);
    }else if(cameraLeft->sensorEnable){
      cameraLeft->updateSensor(hal);
    }
  }else if(camerasStatus != out){
    // If here, it means that the status is changed due to the interface
    // Set the status
    camerasStatus     = out;
    cameraNAOqiStatus = camerasStatus;

    // If status is 0, activate camera
    if(camerasStatus == 0){
      // Activate camera top
      //                if(cameraTop==NULL){
      //                    std::vector<const Sim::CameraSensor*> cameraSensors = model->cameraSensors();
      //                    cameraProxy->setActiveCamera(cameraProxyName,0);
      //                    cameraTop = new v_repNaoSensors::CameraSensor(cameraSensors[1]->name(), vrepId, cameraSensors[1]);
      //                    cameraTop->activeCamera();
      //                    cameraTop->sendResolution(cameraProxy);
      //                    cameraTop->updateSensor(hal);
      //                }else{
      //                    cameraProxy->setActiveCamera(cameraProxyName,0);
      //                    cameraTop->activeCamera();
      //                    cameraTop->sendResolution(cameraProxy);
      //                    cameraTop->updateSensor(hal);
      //                }
      //                if(cameraBottom!=NULL)
      //                    cameraBottom->disableCamera();

      cameraProxy->setActiveCamera(1);   ///MOD BENOIT
      cameraLeftEye->activeCamera();
      cameraLeftEye->sendResolution(cameraProxy);
      //                cameraTop->updateSensor(hal);
      cameraRightEye->disableCamera();
      cameraRight->disableCamera();
      cameraLeft->disableCamera();

    }else if(camerasStatus == 1){
      // Activate camera bottom
      //                if(cameraTop==NULL){
      //                    std::vector<const Sim::CameraSensor*> cameraSensors = model->cameraSensors();
      //                    cameraProxy->setActiveCamera(cameraProxyName,1);
      //                    cameraBottom = new v_repNaoSensors::CameraSensor(cameraSensors[0]->name(), vrepId, cameraSensors[0]);
      //                    cameraBottom->activeCamera();
      //                    cameraBottom->sendResolution(cameraProxy);
      //                    cameraBottom->updateSensor(hal);
      //                }else{
      //                    cameraProxy->setActiveCamera(cameraProxyName,1);
      //                    cameraBottom->activeCamera();
      //                    cameraBottom->sendResolution(cameraProxy);
      //                    cameraBottom->updateSensor(hal);
      //                }
      //                if(cameraTop!=NULL)
      //                    cameraTop->disableCamera();

      cameraProxy->setActiveCamera(0);
      cameraRightEye->activeCamera();
      cameraRightEye->sendResolution(cameraProxy);
      //                cameraBottom->updateSensor(hal);
      cameraLeftEye->disableCamera();
      cameraLeft->disableCamera();
      cameraRight->disableCamera();
    }else{
      // The cameras have to be disabled
      cameraRight->disableCamera();
      cameraLeft->disableCamera();
      cameraLeftEye->disableCamera();
      cameraRightEye->disableCamera();
    }
  }else{
    // NAOqi has switched camera
    camerasStatus     = out2;
    cameraNAOqiStatus = camerasStatus;
    if(camerasStatus == 0){
      // Activate top camera
      //                if(cameraTop==NULL){
      //                    std::vector<const Sim::CameraSensor*> cameraSensors = model->cameraSensors();
      //                    cameraTop = new v_repNaoSensors::CameraSensor(cameraSensors[1]->name(), vrepId, cameraSensors[1]);
      //                }
      cameraTop->activeCamera();
      cameraTop->sendResolution(cameraProxy);
      //                cameraTop->updateSensor(hal);
      cameraBottom->disableCamera();

      // Set the interface
      out3+=256;
      simSetUIButtonProperty(UIHandle, 8, out3);
    }else if(camerasStatus == 1){
      // Activate top camera
      //                if(cameraBottom==NULL){
      //                    std::vector<const Sim::CameraSensor*> cameraSensors = model->cameraSensors();
      //                    cameraBottom = new v_repNaoSensors::CameraSensor(cameraSensors[0]->name(), vrepId, cameraSensors[0]);
      //                }
      cameraBottom->activeCamera();
      cameraBottom->sendResolution(cameraProxy);
      //                cameraBottom->updateSensor(hal);
      cameraTop->disableCamera();

      // Set the interface
      out3-=256;
      simSetUIButtonProperty(UIHandle, 8, out3);
    }
  }

  std::cout << "cameraStatusAfter: " << camerasStatus << std::endl;
}

void v_repRomeoRobot::updateFSRSensors(){
  // Update camera sensors
  std::vector<v_repRomeoSensors::FSRSensor*>::iterator FSRit;
  for(FSRit=FSRSensorsVrep.begin();FSRit!=FSRSensorsVrep.end();++FSRit){
    //std::cout << "NameFSR sensor: " << (*FSRit)->sensorName.c_str() << std::endl;
    (*FSRit)->updateSensor(hal);
  }
}

void v_repRomeoRobot::updateHands(){
  // Update hands
  // Left hand
  if(v_repRomeoLeftHand!=NULL)
    v_repRomeoLeftHand->updateHand(hal);
  // Right hand
  if(v_repRomeoRightHand!=NULL)
    v_repRomeoRightHand->updateHand(hal);
}

void v_repRomeoRobot::killThreads(){
  if(t1 != NULL){
    killNAOqiEx();
    t1->interrupt();
  }
  if(t2 != NULL){
    killHALEx();
    t2->interrupt();
  }

  // Processes
  //kill(pidHAL,SIGTERM);
  //kill(pidNAOqi,SIGTERM);
}

/////////////////////////////////////////
//// Methods of class v_repRomeoRobot /////
/////////////////////////////////////////

v_repRomeoRobot::v_repRomeoRobot(int objHandle){

  //        std::vector<const Sim::AngleActuator*> angleActuators;

  // Save the vrep ID
  vrepId = objHandle;

  // Initialize UINAO to NULL pointer
  UIHandle = -1;

  // Launch the NAOqi launcher
  sim = new Sim::NAOqiLauncher();
  // Init special joint
  //initVariables();

  // Romeo initialization
  initModelInterface();

  // Initialize the threads of NAOqi and HAL
  initThreads();

  std::vector<std::string> namesVrep;
  std::vector<simInt> handlesVrep;

  // Get current Vrep structure
  getVrepStruct(namesVrep, handlesVrep, vrepId);

  // Get the structure from Aldebaran
  getAldebStruct();

  // Find the correspondences between them
  findAldebVrepCorr(namesVrep, handlesVrep);

  // Get the UI handle
  UIHandle = simGetUIHandle(UI_NAME);

  // Init the hands
  initHands();

  ///Benoit put this here instead of before the UIHandle.////
  // Set the initial pose of the robot
  initPose();
}

void v_repRomeoRobot::findCorrespondences(){

  // Inizialize the proxy
  initProxy();

  // Init the sensors
  initSensors();

}

v_repRomeoRobot::~v_repRomeoRobot(){

}

void v_repRomeoRobot::_termination_handler (int){
  //std::cout << "Ending sim!!!!!!" << std::endl;
  sleep(2);
  delete sim;
  delete hal;
  delete model;
  if(t2->joinable())
    t2->interrupt();
  sleep(2);
  if(t1->joinable())
    t1->interrupt();
  exit(0);
}

/////////////////////////////////////////
///// Methods of class v_repRomeoHand /////
/////////////////////////////////////////

v_repRomeoHand::v_repRomeoHand(){
  enableHand = false;
}

v_repRomeoHand::v_repRomeoHand(const Sim::CoupledActuator* ca, const Sim::CoupledSensor* sa,simInt root){
  // Assign the coupled actuator
  couplAct   = ca;
  couplSens  = sa;
  enableHand = true;

  findVrepTree(root);
}

v_repRomeoHand::v_repRomeoHand(const Sim::CoupledActuator* ca,const Sim::Model* mo,simInt root){
  // Assign the coupled actuator
  couplAct   = ca;
  couplSens  = mo->coupledSensor(couplAct->name());
  enableHand = true;

  findVrepTree(root);
}

v_repRomeoHand::~v_repRomeoHand(){}

void v_repRomeoHand::findVrepTree(simInt &objHandle){
  // Take the structure of the robot where is attached the Vrep script
  std::vector<int> toExplore;
  int childHandle, index;
  std::string str;
  simBool cyclic;
  simFloat interval[2];
  toExplore.push_back(objHandle);
  while (toExplore.size()!=0){
    objHandle=toExplore[toExplore.size()-1];
    toExplore.pop_back();
    index=0;
    childHandle=simGetObjectChild(objHandle,index++);
    //            std::cout << "childHandle: " << childHandle << std::endl;
    if(childHandle!=-1){
      str = simGetObjectName(childHandle);
      if(std::string::npos != str.find(PATH_TO_SEARCH)){
        jointName.push_back(simGetObjectName(childHandle));
        jointHandle.push_back(childHandle);
        // Get joint limit
        simGetJointInterval(childHandle,&cyclic,interval);
        jointLimit.push_back(interval[1]);
      }
    }
    while (childHandle!=-1){
      toExplore.push_back(childHandle);
      childHandle=simGetObjectChild(objHandle,index++);
      if(childHandle!=-1){
        str = simGetObjectName(childHandle);
        if(std::string::npos != str.find(PATH_TO_SEARCH)){
          jointName.push_back(simGetObjectName(childHandle));
          jointHandle.push_back(childHandle);
          // Get joint limit
          simGetJointInterval(childHandle,&cyclic,interval);
          jointLimit.push_back(interval[1]);
        }
      }
    }
  }
}

void v_repRomeoHand::setJointValue(const float &val){
  int size = jointHandle.size();
  for(int i=0;i<size;++i){
    simSetJointTargetPosition(jointHandle[i],val*jointLimit[i]);
  }
}

std::string v_repRomeoHand::getJointName(int i) const{
  return jointName[i];
}

simInt v_repRomeoHand::getJointHandle(int i) const{
  return jointHandle[i];
}

std::vector<simInt> v_repRomeoHand::getJointHandles() const{
  return jointHandle;
}

std::vector<std::string> v_repRomeoHand::getJointNames() const{
  return jointName;
}

std::vector<simInt>* v_repRomeoHand::getJointHandlesPtr(){
  return &jointHandle;
}

std::vector<std::string>* v_repRomeoHand::getJointNamesPtr(){
  return &jointName;
}

void v_repRomeoHand::updateHand(Sim::HALInterface *&hal){
  if(enableHand){
    float val = hal->fetchCoupledActuatorValue(couplAct);
    hal->sendCoupledSensorValue(couplSens, val);
    int size = jointHandle.size();
    for(int i=0;i<size;++i){
      simSetJointTargetPosition(jointHandle[i],val*jointLimit[i]);
    }
  }
}

void v_repRomeoHand::disableHand(){
  if(enableHand){
    enableHand = false;
    int size = jointHandle.size();
    for(int i=0;i<size;++i){
      simSetModelProperty(jointHandle[i],sim_modelproperty_not_dynamic+sim_modelproperty_not_respondable);
    }

  }
}

void v_repRomeoHand::activeHand(){
  int size = jointHandle.size();
  std::cout << "jointHandleSize: " << jointHandle.size() << std::endl;
  for(int i=0;i<size;++i){
    std::cout << "jointHandName: " << simGetObjectName(jointHandle[i]) << " number : " << jointHandle[i] << std::endl;
    //simSetModelProperty(jointHandle[i],0);
    simSetJointMode(jointHandle[i],sim_jointmode_motion_deprecated,1);
  }
  enableHand = true;
}

}

