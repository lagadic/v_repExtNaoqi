#!/bin/bash

# Find simulator-sdk home folder
. ./scripts/findSimSDK.sh

. ~/.bashrc
. ~/.profile

STRING_TO_SEARCH="SIMULATOR_SDK_HOME"
foundString=$(grep -i ${STRING_TO_SEARCH} ~/.bashrc)
# If no valid path is found, exit
if [[ "${foundString}" != "" ]]; then
	
	currDir=$(pwd)

	if [ -f CMakeCache.txt ];
	then
	   rm CMakeCache.txt
	fi
	
	cmake -DCMAKE_TOOLCHAIN_FILE=${SIMULATOR_SDK_HOME}/toolchain-pc.cmake .
	make

	cp ${SIMULATOR_SDK_HOME}/lib/libROMEOAD.so ${currDir}
	mv libROMEOAD.so ../../libv_repExtRomeo.so
else
	echo "SIMULATOR_SDK_HOME not found. Please fix it or launch scripts/findSimSDK.sh"
fi
