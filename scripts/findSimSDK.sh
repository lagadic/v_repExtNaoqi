#!/bin/bash

STRING_TO_SEARCH="SIMULATOR_SDK_HOME"

foundString=$(grep -i ${STRING_TO_SEARCH} ~/.bashrc)
#echo -e "foundString = "${foundString}

if [[ "${foundString}" == "" ]]; then
	pathSDK=`find ~ -name alnaosim.h`
	set -- $pathSDK
	for i in $@
	do
		# Find current guess of path where is SDK
		currPath=$i
		# Erase /include/alnaosim/alnaosim.h that is in every folder where alnaosim.h is (28 characters)
		substring=${currPath:0:end-28}
	
		# Check if the current is the right value for the SIMULATOR_SDK_HOME	
		echo -e "I found \033[1mSIMULATOR_SDK_HOME\033[0m = "${substring}
		tput sgr0
		read -p "Is it correct? (y/n):"
		while [ "$REPLY" != "y" ] && [ "$REPLY" != "n" ]
		do
			echo -e '\E[47;31m'"\033[1mJust answer y or n, dude!\033[0m"
			tput sgr0
			read -p "Is it correct? (y/n):"
		done
		if [ "$REPLY" == "y" ]; then
			echo "Good, thanks!"
			# Insert in .bashrc
			export SIMULATOR_SDK_HOME=$substring
			echo "# SIMULATOR_SDK_HOME" >> ~/.bashrc
			echo "export SIMULATOR_SDK_HOME=$substring" >> ~/.bashrc
			echo "# SIMULATOR_SDK_HOME" >> ~/.profile
			echo "export SIMULATOR_SDK_HOME=$substring" >> ~/.profile
			echo "SIMULATOR_SDK_HOME exported in your bashrc. Thank you for using our software!"
			break
		fi
		if [ "$REPLY" == "n" ]; then
			echo " Let's try another path"
		fi
	done
	
	if [ "$REPLY" != "y" ]; then
		#If no valid path has been found, try to fix manually
		echo " I'm sorry about that. I did not succeed to find a valid path for your simulator-sdk. Let's fix it together."
		read -p " Please insert the correct path:"
		substring=$REPLY
		export SIMULATOR_SDK_HOME=$substring
		echo "# SIMULATOR_SDK_HOME" >> ~/.bashrc
		echo "export SIMULATOR_SDK_HOME=$substring" >> ~/.bashrc
		echo "# SIMULATOR_SDK_HOME" >> ~/.profile
		echo "export SIMULATOR_SDK_HOME=$substring" >> ~/.profile
		echo "SIMULATOR_SDK_HOME exported in your bashrc. Thank you for using our software!"
	fi
else 
	echo "SIMULATOR_SDK_HOME found!"
fi
