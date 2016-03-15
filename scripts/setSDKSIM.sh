#!/bin/bash
# Find naoqi-sdk home folder
STRING_TO_SEARCH="SIMULATOR_SDK_HOME"

foundString=$(grep -i ${STRING_TO_SEARCH} ~/.bashrc)
#echo -e "foundString = "${foundString}

if [[ "${foundString}" == "" ]]; then
	SIMSDK_HOME=$(pwd)
	# Insert in .bashrc
	export SIMULATOR_SDK_HOME=$substring
	echo "# SIMULATOR_SDK_HOME" >> ~/.bashrc
	echo "export SIMULATOR_SDK_HOME=$SIMSDK_HOME" >> ~/.bashrc
	echo "# SIMULATOR_SDK_HOME" >> ~/.profile
	echo "export SIMULATOR_SDK_HOME=$SIMSDK_HOME" >> ~/.profile
	echo "SIMULATOR_SDK_HOME exported in your bashrc"
fi
