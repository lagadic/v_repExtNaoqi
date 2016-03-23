// Written by Marco Cognetti

#include "v_repExtRomeoSensors.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

namespace v_repRomeoSensors{

#define IMU_FORCE "force"
#define IMU_MASS  "mass"
#define FSR_MASS  "Mass"
// #define NAO_MASS  5.374
#define ROMEO_MASS  40.53

///////////////////////////////////////////////////
//////////////////// Sensor ///////////////////////
///////////////////////////////////////////////////

Sensor::Sensor(){
  sensorName     = "";
  sensorHandle   = -1;
  sensorEnable   = false;
  lastUpdateTime = 0.0;
}

Sensor::Sensor(const std::string &name, simInt handle){
  sensorName = name;
  searchSensorHandle(name,handle);
  if(sensorHandle==-1)
    sensorEnable = false;
  else
    sensorEnable = true;
}

Sensor::Sensor(const simInt &handle){
  sensorHandle = handle;
  sensorName   = simGetObjectName(sensorHandle);
  sensorEnable = true;
}

Sensor::Sensor(const simInt &name, std::vector< std::pair<std::string, simInt> > &namesVrep){
  // This method computes the correspondence between the given name
  // and the found in Vrep tree
  bool foundCorr    = false;
  int namesVrepSize = namesVrep.size();
  int i = 0;
  for(i=0;i<namesVrepSize;++i){
    if (std::string::npos != namesVrep[i].first.find(name)){
      sensorHandle  = namesVrep[i].second;
      foundCorr   = true;
      sensorName  = namesVrep[i].second;
      break;
    }
  }
  // If a corrispondence is not found
  if(!foundCorr){
    sensorHandle = -1;
    //std::cout << "WARNING: " << namesVrep[i].first << " not found!" << std::endl;
    sensorName = "";
    sensorEnable = false;
  }else
    sensorEnable = true;

}

Sensor::~Sensor(){}

Sensor::Sensor(const Sensor &A){
  sensorName      = A.sensorName;
  sensorHandle    = A.sensorHandle;
  sensorEnable    = A.sensorEnable;
  sensorChildrens = A.sensorChildrens;
}

Sensor& Sensor::operator=(const Sensor& A){
  if(this!=&A){
    sensorName      = A.sensorName;
    sensorHandle    = A.sensorHandle;
    sensorEnable    = A.sensorEnable;
    sensorChildrens = A.sensorChildrens;
  }
  return *this;
}

void Sensor::findVrepTree(simInt &objHandle, std::vector< std::pair< std::string,simInt> > &out){
  int childHandle, index=0;
  // Take the structure of the robot where is attached the Vrep script
  std::pair< std::string,simInt> temp;
  std::vector<int> toExplore;
  // toExplore include the handles to be explored
  toExplore.push_back(objHandle);
  while (toExplore.size()!=0){
    objHandle=toExplore[toExplore.size()-1];
    toExplore.pop_back();
    index = 0;
    childHandle=simGetObjectChild(objHandle,index++);
    if(childHandle!=-1){
      temp.first  = simGetObjectName(childHandle);
      temp.second = childHandle;
      out.push_back(temp);
    }
    while (childHandle!=-1){
      toExplore.push_back(childHandle);
      childHandle=simGetObjectChild(objHandle,index++);
      if(childHandle!=-1){
        temp.first  = simGetObjectName(childHandle);
        temp.second = childHandle;
        out.push_back(temp);
      }
    }
  }
}

void Sensor::findVrepTree(simInt &objHandle, std::vector< simInt > &out){
  int childHandle, index=0;
  // Take the structure of the robot where is attached the Vrep script
  std::vector<int> toExplore;
  toExplore.push_back(objHandle);
  while (toExplore.size()!=0){
    objHandle=toExplore[toExplore.size()-1];
    toExplore.pop_back();
    index = 0;
    childHandle=simGetObjectChild(objHandle,index++);
    if(childHandle!=-1){
      out.push_back(childHandle);
    }
    while (childHandle!=-1){
      toExplore.push_back(childHandle);
      childHandle=simGetObjectChild(objHandle,index++);
      if(childHandle!=-1){
        out.push_back(childHandle);
      }
    }
  }
}

void Sensor::searchSensorHandle(const std::string &name, simInt handle){
  sensorHandle = simGetObjectHandle(name.c_str());
  if(sensorHandle != -1){
    sensorName = name;
  }else{
    // Search for current sensor
    findVrepTree(handle,sensorChildrens);
    int size = sensorChildrens.size();
    for(int i=0;i<size;++i){
      if (std::string::npos != sensorChildrens[i].first.find(name)){
        sensorHandle = sensorChildrens[i].second;
        sensorName = sensorChildrens[i].first;
        break;
      }
    }

    // Get current sensor tree
    sensorChildrens.clear();
    simInt sensorHandleTemp = sensorHandle;
    if(sensorHandle!=-1){
      findVrepTree(sensorHandleTemp,sensorChildrens);
    }
  }
}

void Sensor::updateSensorRate(){
  simFloat acTime = simGetSimulationTime();
  if(acTime - lastUpdateTime >= updateStep){
    updateSensor();
    lastUpdateTime = acTime;
  }
}

///////////////////////////////////////////////////
/////////////// InertialSensor ////////////////////
///////////////////////////////////////////////////

InertialSensor::InertialSensor(const std::string &name) : Sensor(name) {
  // Initialize handles at invalid pointers
  massHandle = -1; forceHandle = -1;

  if(sensorHandle!=-1){

  }
  IMUdata.resize(7);
}

InertialSensor::InertialSensor(const std::string &name, simInt handle, const Sim::InertialSensor* simSDKInSensPt) : Sensor (name,handle) {
  // Initialize handles at invalid pointers
  massHandle = -1; forceHandle = -1;

  // Initialize the simulatorSDK pointer to inertial sensor
  simSDKPt = simSDKInSensPt;

  // Get the mass of the main object
  float inMatrix[9];
  float COG[3];
  std::vector< std::pair<std::string, simInt> >::iterator it;

  // Get the handle to the mass and force sensor
  for(it=sensorChildrens.begin();it!=sensorChildrens.end();++it){
    if (std::string::npos != it->first.find(IMU_MASS)){
      simGetShapeMassAndInertia(it->second, &mass,inMatrix,COG,NULL);
      massHandle = it->second;
    }else if (std::string::npos != it->first.find(IMU_FORCE)){
      forceHandle = it->second;
    }
  }
  IMUdata.resize(7);
}

std::vector<float> InertialSensor::getInertialSensorData() const{
  return IMUdata;
}

std::vector<float>* InertialSensor::getInertialSensorDataPtr(){
  return &IMUdata;
}

void InertialSensor::setInertialSensorData(std::vector<float> &data){
  IMUdata = data;
}

float InertialSensor::getMass() const{
  return mass;
}

void InertialSensor::setMass(const float m){
  mass = m;
  if(massHandle!=-1){
    float* inMatrix = new float(9);
    float* COG      = new float(3);
    float tempMass;
    // Get other data I do not want to modify
    simGetShapeMassAndInertia(massHandle, &tempMass,inMatrix,COG,NULL);
    simSetShapeMassAndInertia(massHandle,mass,inMatrix,COG,NULL);
  }
}

simInt InertialSensor::getMassHandle() const{
  return massHandle;
}

void InertialSensor::setMassHandle(const simInt &hm){
  massHandle = hm;
}

simInt InertialSensor::getForceHandle() const{
  return forceHandle;
}

void InertialSensor::setForceHandle(const simInt &hf){
  forceHandle = hf;
}

void InertialSensor::updateSensor(){
  float temp[3];

  // Orientation
  // Get the orientation matrix
  simGetObjectMatrix(massHandle, -1, tempOri);

  // Get the current orientation and copy into data
  simGetEulerAnglesFromMatrix(tempOri,temp);
  IMUdata[0] = temp[0];
  IMUdata[1] = temp[1];

  // Invert the matrix and get the current absolute angular velocity
  simInvertMatrix(tempOri);
  simGetObjectVelocity(sensorHandle,NULL,temp);

  // Get the angular velocity in body frame
  // I need only the omegaX and omegaY (i<2)
  //simGetVelocity(massHandle, NULL, temp);
  for(int i = 0;i<2;++i)
    for(int j = 0;j<3;++j)
      IMUdata[5+i] += tempOri[i+4*j] * temp[i];

  // Read the force sensor data
  simInt out = simReadForceSensor(forceHandle,temp,NULL);
  // Acceleration
  IMUdata[2] = temp[0]/mass;
  IMUdata[3] = temp[1]/mass;
  IMUdata[4] = temp[2]/mass;

  //        std::cout << "IMURaw: " << simGetObjectName(forceHandle) << " " << out << " " << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
  //        std::cout << "IMU: " << simGetObjectName(forceHandle) << " " << out << " " << temp[0]/mass << " " << temp[1]/mass << " " << temp[2]/mass << std::endl;

  //for(int i=0;i<IMUdata.size();++i)
  //std::cout << "IMUData[" << i << "]: " << IMUdata[i] << std::endl;
}

void InertialSensor::updateSensor(Sim::HALInterface* &hal){
  updateSensor();
  hal->sendInertialSensorValues(simSDKPt,IMUdata);
}

///////////////////////////////////////////////////
//////////////// CameraSensor /////////////////////
///////////////////////////////////////////////////

CameraSensor::CameraSensor(const std::string &name) : Sensor(name) {

}

CameraSensor::CameraSensor(const std::string &name, simInt handle, const Sim::CameraSensor* simSDKInSensPt) : Sensor (name,handle) {
  // Initialize the simulatorSDK pointer to inertial sensor
  simSDKPt = simSDKInSensPt;

  // Get the width and height of the image
  int test = simGetVisionSensorResolution(sensorHandle,widthHeight);

  std::cout << "Width and height of the image : " << test << std::endl;
  std::cout << "Width of the image : " << widthHeight[0] << std::endl;
  std::cout << "Height of the image : " << widthHeight[1] << std::endl;

  // Total dimension of the image seen by the robot (the last 3 is for RGB channel)
  totDim = widthHeight[0] * widthHeight[1] * 3;

  // Send to NAOqi the resolution of the camera
  // SET THE CAMERA RESOLUTION
  //        cameraProxy = new AL::ALVideoDeviceProxy("127.0.0.1");

  //        sendResolution();

  //        std::cout << "activeCamera: " << cameraProxy->getActiveCamera() << std::endl;

  // Set the starting image dimension
  img = (unsigned char*) new unsigned char[totDim];

  //std::cout << "Width of the camera: " << widthHeight[0] << " height: " << widthHeight[1] << std::endl;
}

void CameraSensor::sendResolution(AL::ALVideoDeviceProxy* &cameraProxy){
  switch(totDim){
  case 57600: // 160x120x3 (RGB)
    cameraProxy->setParam(14,0);
    break;
  case 230400: // 320x240x3 (RGB)
    cameraProxy->setParam(14,1);
    break;
  case 921600: // 640x480x3 (RGB)
    cameraProxy->setParam(14,2);
    break;
  case 3686400:// 1280x960x3 (RGB)
    cameraProxy->setParam(14,3);
    break;
  }
}

void CameraSensor::updateSensor(){

  imgTmp = simGetVisionSensorImage(sensorHandle);

  // Converting image
  //        img = (unsigned char*) imgTmp;

  //std::cout << "convTempValue" << std::endl;
  ////std::cout << img[0] << std::endl;

  //        std::cout << "activeCamera: " << cameraProxy->getActiveCamera() << std::endl;
}

void CameraSensor::updateSensor(Sim::HALInterface* &hal){
  if(sensorEnable){
    updateSensor();

    for(int i=0;i<totDim;i=i+3){
      // Invert and convert the image from [0,1] range to [0, 255] RGB scale
      img[i]  = imgTmp[i]*255;
      img[i+1]  = imgTmp[i+1]*255;
      img[i+2]  = imgTmp[i+2]*255;

      // From OpenCV transformation
      //            img[i]   = imgTmp[i]*255;
      //            img[i+1] = imgTmp[i+1]*255;
      //            img[i+2] = imgTmp[i+2]*255;

      //            //std::cout << "ImgTmp[" << i << "]: " << imgTmp[i] << std::endl;
      //            imgToSend[320*240*3-i-1] = imgTmp[i]*255;
      //            if(count==0){
      //                imgToSend[320*240*3-i-1] = imgTmp[i+2]*255;
      //                count++;
      //            }else if(count==1){
      //                imgToSend[320*240*3-i-1] = imgTmp[i+1]*255;
      //                count++;
      //            }else{
      //                imgToSend[320*240*3-i-1] = imgTmp[i]*255;
      //                count = 0;
      //            }
      //            //std::cout << "Img[" << i << "]: " << imgToSend[i] << std::endl;
    }

    //        openCVVisualizer();

    std::cout << "Expected camera size: " << hal->cameraBufferSize(simSDKPt) << std::endl;

    hal->sendCameraSensorValue(simSDKPt,img);
  }
}

void CameraSensor::openCVVisualizer(){
  // OpenCV visualization
  int height = 120;
  int width = 160;
  cv::Mat imageOpenCV(height,width,CV_32FC3);

  //        for(int i=0;i<height;i++){
  //            for(int j=0;j<width;j++){
  //                imageOpenCV.at<cv::Vec3f>(i,j)[0] = imgTmp[i*width+j]*255;
  //                imageOpenCV.at<cv::Vec3f>(i,j)[1] = imgTmp[width*height+i*width+j]*255;
  //                imageOpenCV.at<cv::Vec3f>(i,j)[2] = imgTmp[width*height*2+i*width+j]*255;
  //            }
  //        }

  //        for(int i=0;i<height;i++){
  //            for(int j=0;j<width;j++){
  //                imageOpenCV.at<cv::Vec3f>(i,j)[0] = imgTmp[i+j*height]*255;
  //                imageOpenCV.at<cv::Vec3f>(i,j)[1] = imgTmp[width*height+i+j*height]*255;
  //                imageOpenCV.at<cv::Vec3f>(i,j)[2] = imgTmp[width*height*2+i+j*height]*255;
  //            }
  //        }

  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      //                imageOpenCV.at<cv::Vec3f>(i,j)[0] = (float)img[totDim-(3*(i*width+j)+0)];
      //                imageOpenCV.at<cv::Vec3f>(i,j)[1] = (float)img[totDim-(3*(i*width+j)+1)];
      //                imageOpenCV.at<cv::Vec3f>(i,j)[2] = (float)img[totDim-(3*(i*width+j)+2)];
      imageOpenCV.at<cv::Vec3f>(i,j)[0] = (float)img[(3*(i*width+j)+0)]/255;
      imageOpenCV.at<cv::Vec3f>(i,j)[1] = (float)img[(3*(i*width+j)+1)]/255;
      imageOpenCV.at<cv::Vec3f>(i,j)[2] = (float)img[(3*(i*width+j)+2)]/255;
    }
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
  cv::imshow( "Display window", imageOpenCV );
}

void CameraSensor::disableCamera(){
  sensorEnable = false;
  simSetExplicitHandling(sensorHandle,1);
}

void CameraSensor::activeCamera(){
  sensorEnable = true;
  simSetExplicitHandling(sensorHandle,0);
}

////////////////////////////////////////////////
///////////////// FSR Sensors //////////////////
////////////////////////////////////////////////
FSRSensor::FSRSensor(const std::string &name) : Sensor(name) {
  float accel[3];
  // Get the gravity vector and take the z-component
  simGetArrayParameter(sim_arrayparam_gravity,accel);
  gravity = accel[2];

  massTot = ROMEO_MASS;
}

FSRSensor::FSRSensor(const std::string &name, simInt handle, const Sim::FSRSensor* simSDKInSensPt) : Sensor (name,handle) {
  // Initialize the simulatorSDK pointer to inertial sensor
  simSDKPt = simSDKInSensPt;

  massTot = ROMEO_MASS;

  // Get the gravity intensity
  float accel[3];
  simGetArrayParameter(sim_arrayparam_gravity,accel);
  gravity = accel[2];

  // Get the mass of the main object
  float inMatrix[9];
  float COG[3];

  std::vector< std::pair<std::string, simInt> >::iterator it;
  for(it=sensorChildrens.begin();it!=sensorChildrens.end();++it){
    if (std::string::npos != it->first.find(FSR_MASS)){
      simGetShapeMassAndInertia(it->second, &mass,inMatrix,COG,NULL);
      break;
    }
  }
}

void FSRSensor::updateSensor(){
  float temp[3];
  simInt out = simReadForceSensor(sensorHandle,temp,NULL);

  // I need the force along z
  forceZ = temp[2];

  // Compute the mass distributed to each FSR sensor (the 8 since they are 8 FSRs)
  massZ = forceZ * massTot / (mass * gravity * 8);

  std::cout << "mass: " << mass << " massTot: " << massTot << " gravity: " << gravity << std::endl;
  std::cout << "massZ: " << massZ << " for " << simGetObjectName(sensorHandle) << std::endl;
}

void FSRSensor::updateSensor(Sim::HALInterface* &hal){
  updateSensor();
  // TODO: ADD HAL
  hal->sendFSRSensorValue(simSDKPt,massZ);
}
}

