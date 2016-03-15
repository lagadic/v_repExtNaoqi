#!/bin/bash

echo "Searching naoqi-bin. This might takes a while..."
pathSDK=`find ~ -name naoqi-bin`
set -- $pathSDK
for i in $@
do
	# Find current guess of path where is NAOQI-SDK
	currPath=$i
	# Erase /bin/naoqi-bin that is in every folder where /bin/naoqi-bin is (14 characters)
	substring=${currPath:0:end-14}

	# Check if the current is the right value for the NAOQI_SDK_HOME	
	echo -e "I found \033[1mNAOQI_SDK_HOME\033[0m = "${substring}
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
		export NAOQI_SDK_HOME=$substring
		echo "# NAOQI_SDK_HOME" >> ~/.bashrc
		echo "export NAOQI_SDK_HOME=$substring" >> ~/.bashrc
		echo "# NAOQI_SDK_HOME" >> ~/.profile
		echo "export NAOQI_SDK_HOME=$substring" >> ~/.profile
		echo "NAOQI_SDK_HOME exported in your bashrc. Thank you for using our software!"
		break
	fi
	if [ "$REPLY" == "n" ]; then
		echo " Let's try another path"
	fi
done

if [ "$REPLY" != "y" ]; then
	#If no valid path has been found, try to fix manually
	echo " I'm sorry about that. I did not succeed to find a valid path for your naoqi-sdk. Let's fix it together."
	read -p " Please insert the correct path:"
	substring=$REPLY
	export NAOQI_SDK_HOME=$substring
	echo "# NAOQI_SDK_HOME" >> ~/.bashrc
	echo "export NAOQI_SDK_HOME=$substring" >> ~/.bashrc
	echo "# NAOQI_SDK_HOME" >> ~/.profile
	echo "export NAOQI_SDK_HOME=$substring" >> ~/.profile
	echo "NAOQI_SDK_HOME exported in your bashrc. Thank you for using our software!"
fi
