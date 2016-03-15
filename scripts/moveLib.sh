#!/bin/bash

# Find naoqi-sdk home folder
STRING_TO_SEARCH="NAOQI_SDK_HOME"

foundString=$(grep -i ${STRING_TO_SEARCH} ~/.bashrc)
#echo -e "foundString = "${foundString}

if [[ "${foundString}" == "" ]]; then
	. ./findNAOQISDK.sh
fi

# Reload temporary files
. ~/.bashrc
. ~/.profile

STRING_TO_SEARCH="NAOQI_SDK_HOME"
foundString=$(grep -i ${STRING_TO_SEARCH} ~/.bashrc)
# If no valid path is found, exit
if [[ "${foundString}" != "" ]]; then
	# Away from scripts to main folder of simulator-sdk
	cd ..
	SIMSDK_HOME=$(pwd)
	. ./scripts/setSDKSIM.sh
	
	# Here copy the executable of NAOqi
	cp ${NAOQI_SDK_HOME}/bin/naoqi-bin bin/naoqi-ex

	# Here copy the needed libraries from NAOqi
	cp ${NAOQI_SDK_HOME}/lib/naoqi/libdcm_hal.so lib/naoqi/
	cp ${NAOQI_SDK_HOME}/lib/liblib_dcm.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libhal_common.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libhal_core.so lib/
	cp ${NAOQI_SDK_HOME}/lib/naoqi/libalnavigationinterface.so lib/naoqi/
	cp ${NAOQI_SDK_HOME}/lib/libdcm_plugins.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_almemory.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_timedcommand.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libnao_devices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libnavcommon.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_preferences.so lib/
	cp ${NAOQI_SDK_HOME}/lib/librobot_devices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libromeo_devices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/naoqi/libnavigation.so lib/naoqi/
	cp ${NAOQI_SDK_HOME}/lib/libnaointerface.so lib/
	cp ${NAOQI_SDK_HOME}/lib/liblocalnavigation.so lib/
	cp ${NAOQI_SDK_HOME}/lib/liblaser_utils.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libnaosensorutils.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libmetrical.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libpathsimplifier.so lib/

	# Here copy the executable for hal
	cp ${NAOQI_SDK_HOME}/bin/hal bin/hal-ex

	# Here copy the needed libraries from HAL
	cp ${NAOQI_SDK_HOME}/lib/libnao_running.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_test.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_iocommunication.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_initmotorboard.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_actuatorifnostiffness.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_fsrtotalcenterofpression.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_testrobotversion.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_maxcurrent.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_specialjointlimitation.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_clientsync.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_initnaodevices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_motortemperature.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_calibration.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_ledifnodcm.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_simulation.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_simulation_fill_attributes.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_naospecial.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_naoavailabledevice.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libnao-modules.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libio_headusb.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libio_headi2c.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libnaospecialsimulation_running.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_addnaodevicesspecialsimulation.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libromeo_running.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_initromeodevices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_test-romeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_initromeomotorboard.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_virtualdeviceromeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_motortojoint.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_security-romeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_differentialjointlimitation.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_resendConfigurationToCard.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_initTestBenchDevice.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_debugromeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_actuatorifnostiffnessromeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_simulation_fill_attributes_romeo.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libromeo-modules.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libio-rs485.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libromeo_head_running.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin-initromeoheaddevices.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libplugin_eyescontrolloop.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libio-usb.so lib/
	cp ${NAOQI_SDK_HOME}/lib/libio-usb.so lib/
	if [ ! -d etc/hal ]; then
		mkdir etc/hal
	fi
	cp ${NAOQI_SDK_HOME}/etc/hal/hal.xml etc/hal/

else
	echo "NAOQI_SDK_HOME not found. Please fix it or launch scripts/findNAOQISDK.sh"
fi
