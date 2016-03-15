#ifndef _V_REP_ROMEO
#define _V_REP_ROMEO

#include "v_repExtRomeoSensors.h"

namespace v_repRomeo{
    class v_repRomeoHand{
        private:

        /// \brief This vector contains the names of the Vrep links associated with the hands
        /// It is made up by 8 joints:
        /// thumb: 2 joints, one phalanx (close to the hand) and one 2nd phalanx
        /// middle finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// third finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        std::vector<std::string> jointName;

        /// \brief This vector contains the handles of the Vrep links associated with the hands
        /// It is made up by 8 handles to the joints:
        /// thumb: 2 joints, one phalanx (close to the hand) and one 2nd phalanx
        /// middle finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// third finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        std::vector<int> jointHandle;

        /// \brief Vector that stores the limit of each joint
        std::vector<simFloat> jointLimit;

        /// \brief Coupled actuator from Aldebaran libraries
        const Sim::CoupledActuator* couplAct;

        /// \brief Coupled sensor from Aldebaran libraries
        const Sim::CoupledSensor* couplSens;

        /// \brief Actual gripper position
        float actPos;

        public:

        /// \brief Enable the hand
        bool enableHand;

        /// \brief Default constructor
        v_repRomeoHand();

        /// \brief Root where one can find the tree of the hand
        /// \param[in].ca Pointer to coupled actuator of Aldebaran libraries
        /// \param[in].root Root handle from which the sw founds the fingers
        v_repRomeoHand(const Sim::CoupledActuator* ca, const Sim::CoupledSensor* sa, simInt root);

        /// \brief Root where one can find the tree of the hand
        /// \param[in].ca Pointer to coupled actuator of Aldebaran libraries
        /// \param[in].root Root handle from which the sw founds the fingers
        v_repRomeoHand(const Sim::CoupledActuator* ca, const Sim::Model* mo, simInt root);

        /// \brief Default destructor
        ~v_repRomeoHand();

        void findVrepTree(simInt &objHandle);

        /// \brief Set the joint value to every of each 8 joint
        /// \param[in].&val The value to be set. This value has to be in the interval [0 1]
        void setJointValue(const float &val);

        /// \brief Get the name of the joint. Recall the list of joint:
        /// thumb: 2 joints, one phalanx (close to the hand) and one 2nd phalanx
        /// middle finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// third finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// \param[in].i i-th finger to be get
        std::string getJointName(int i) const;

        /// \brief Get the name of the handle. Recall the list of joint:
        /// thumb: 2 joints, one phalanx (close to the hand) and one 2nd phalanx
        /// middle finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// third finger: 3 joints, one phalanx (close to the hand), one 2nd phalanx and one 3rd phalanx
        /// \param[in].i i-th handle to be get
        simInt getJointHandle(int i) const;

        /// Get the handles to the joints
        std::vector<simInt> getJointHandles() const;

        /// Get the names of the joints
        std::vector<std::string> getJointNames() const;

        /// Get the pointer to handles to the joints
        std::vector<simInt>* getJointHandlesPtr();

        /// Get the pointer to names of the joints
        std::vector<std::string>* getJointNamesPtr();

        void updateHand(Sim::HALInterface* &hal);

        /// \brief Method to disable the hand joints
        void disableHand();

        /// \brief Method to activate the hand joints
        void activeHand();
    };

    class v_repRomeoRobot{
     private:
        // Threads
        /// \brief NAOqi executable thread
        boost::thread* t1;
        /// \brief HAL executable thread
        boost::thread* t2;
        /// \brief Handles to joints
        std::vector<simInt> jointsVrep;
        /// \brief Handles to hand
        std::vector<simInt> handsVrep;
        // Special joint: from NAOqi it is set to nan. I have to reconstruct its value
        simInt rHipYawPitchJoint;
        simInt lHipYawPitchJoint;
/*************************----MODMARCO----*****************************/
	simInt lShoulderPitchJoint;
/*************************----MODMARCO----*****************************/
        // It stores the last valid command for left hip yawPitch joint. I use this value to the right hip yawPitch joint (from Aldebaran set as nan)
        float lastValidlHipYawPitchCommand;
        std::vector<Sim::AngleSensor*> angSensor;
        // String that gets the path of simulator-SDK as environment variable
        char* simSDKHome;
        /// \brief ID of the robot
        int vrepId;
        /// \brief It stores the list of the joint actuators
        std::vector<const Sim::AngleActuator*> angleActuators;
        /// \brief Camera proxy to access the camera
        AL::ALVideoDeviceProxy *cameraProxy;
        std::string cameraProxyName;

        /// \brief Handle to the GUI, if present
        simInt UIHandle;

        // Utility methods
        /// \brief This method is called when one wants to stop the threads
        void _termination_handler(int);

        /// \brief Method that launches an executable (based on sytem)
        /// \param[in].pathToEx Path to the executable to be launched
        void launchEx(const char* pathToEx);

        /// \brief Method that launches an executable (based on sytem)
        /// \param[in].pathToEx Path to the executable to be launched
        void launchExHAL(std::string pathToEx);

        /// \brief Method that kills the executable associated with NAOqi
        /// \details This method is system-dependent
        void killNAOqiEx();

        /// \brief Method that kills the executable associated with HAL
        /// \details This method is system-dependent
        void killHALEx();

        /// \brief Method that kills the executable associated with NAOqi and HAL
        void killEx();

        /// \brief Method that initializes the variables for launching the executables
        void initModelInterface();

        // Init methods
        /// \brief Method that extract all the children starting from objHandle in input
        /// \param[in].&objHandle Handle from which the search starts
        /// \param[out].&names List of the children names
        /// \param[out].&handles List of the children handles
        void findVrepTree(int &objHandle, std::vector<std::string> &names, std::vector<simInt> &handleNames);

        /// \brief Get the current Vrep structure of the robot
        /// \param[in].&objHandle Handle from which the search starts
        /// \param[out].&names List of the children names
        /// \param[out].&handles List of the children handles
        void getVrepStruct(std::vector<std::string>&, std::vector<simInt>&, int &objHandle);

        /// \brief Get the pointer to the actuators from Aldebaran libraries
        void getAldebStruct();

        /// \brief Finds the correspondences between Vrep and Aldebaran joints
        /// \param[in].&namesVrep Vector of Vrep names
        /// \param[in].&handlesVrep Vector of Vrep handles
        /// \details This method takes the names of the joints in Vrep and the names expected from Aldebaran
        /// and matches them. If no corresponce is found for a joint, -1 is added to preserve the length of the list
        void findAldebVrepCorr(std::vector<std::string>&, std::vector<simInt>&);

        /// \brief Start the threads
        void initThreads();

        /// \brief Initialize all the sensors
        void initSensors();

        /// \brief Initialize the inertial sensors (aka IMU)
        /// \details It searches the corresponces between the name given as input and the Vrep simulated sensor
        void initInertialSensors();

        /// \brief Initialize the camera sensor(s)
        /// \details Find the correspondence between the camera name from Aldebaran and the sensor name in Vrep.
        /// Note that only one camera can be active at time
        /// \todo Take into account the case without an interface in Vrep
        void initCameraSensors();

        /// \brief Initialize the FSR sensor
        /// \details Find the correspondence between the FSR names from Aldebaran and the sensor names in Vrep
        /// \todo Take into account the case without an interface in Vrep
        void initFSRSensors();

        /// \brief Initialize the hands
        /// \details Find the correspondence between the hand name from Aldebaran and the name in Vrep
        /// \todo Take into account the case without an interface in Vrep
        void initHands();

        /// \brief Initialize the first pose (initial values of the joints of the robot)
        /// \details It sends the value to the HAL
        void initPose();
/*************************----MODMARCO----*****************************/
	void initPoseRomeo();
/*************************----MODMARCO----*****************************/

        /// \brief Initialize some variables to their initial values
        void initVariables();

        /// \brief Initialize the proxy
        void initProxy();

        /// \brief It return the current state of the cameras
        /// \param[in].&a Bit connected to top camera
        /// \param[in].&b Bit connected to bottom camera
        /// \return 0 if top camera is active, 1 if bottom camera is active, 2 if no camera is active
        int computeCameraStatus(int &a, int &b) const;

        // Update methods
        /// \brief Send the actual value of the sensor to HAL and set the joint value from Aldebaran inputs
        void updatePose();

        /// \brief Get the current value of inertial sensor. It takes the simulated sensor data and send it to HAL
        void updateInertialSensors();

        /// \brief Get the current value of camera sensor. It takes the simulated sensor data and send it to HAL
        void updateCameraSensors();

        /// \brief Get the current value of FSR sensor. It takes the simulated sensor data and send it to HAL
        void updateFSRSensors();

        /// \brief Wrapper method to update all the simulated sensors
        void updateSensors();

        /// \brief Get the current value of the joint of the hands. It takes the simulated sensor data and send it to HAL
        void updateHands();

        /// \brief Method that checks if the converted n-th bit of a given number c is 1 or 0
        /// \param[in].&c Integer whose n-th bit has to be evaluated
        /// \param[in].n Number of the bit to be converted
        /// \return 1 if the n-th bit is set; 0 otherwise
        int isNthBitSet (int &c, int n);

     public:
        /// \brief Pointer to NAOqiLauncher
        Sim::NAOqiLauncher* sim;

        /// \brief Pointer to model from Aldebaran
        Sim::Model* model;

        /// \brief Pointer to HAL interface from Aldebaran. HAL interface is the communication level
        Sim::HALInterface* hal;

        /// \brief Vector of the inertial sensors
        std::vector<v_repRomeoSensors::InertialSensor*> inertialSensorsVrep;

        /// \brief Vector of inertial sensors from Aldebaran (the ones get by the Aldebaran model)
        std::vector<const Sim::InertialSensor*> inertialSensors;
//        std::vector<v_repRomeoSensors::CameraSensor*> cameraSensorsVrep;

        /// \brief Pointer to the top camera
        v_repRomeoSensors::CameraSensor* cameraTop;

        /// \brief Pointer to the bottom camera
        v_repRomeoSensors::CameraSensor* cameraBottom;

        /// \brief Vector of the FSR sensors with the same order as Aldebaran libraries
        std::vector<v_repRomeoSensors::FSRSensor*> FSRSensorsVrep;

        /// \brief Pointer to the left hand
        v_repRomeoHand* v_repRomeoLeftHand;

        /// \brief Pointer to the right hand
        v_repRomeoHand* v_repRomeoRightHand;

        /// \brief Store the actual state of the camera from Vrep
        int camerasStatus;

        /// \brief Store the actual state of the camera from Aldebaran libraries
        int cameraNAOqiStatus;

        /// \brief Default constructor
        /// \param[in].objHandle Handle of the root of the robot in Vrep
        v_repRomeoRobot(int objHandle);

        /// \brief Wrapper function that initializes the proxy and the sensor
        void findCorrespondences();

        /// \brief Default destructor
        ~v_repRomeoRobot();

        /// \brief Wrapper function that updates the joints, the hands and the sensors
        void update();

        /// \brief It interrupts the threads and kill the executables
        void killThreads();
    };
}
#endif

