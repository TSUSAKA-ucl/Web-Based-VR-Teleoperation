#!/usr/bin/bash
Here=`pwd -P`
cd `dirname "$0"` && ThisDir=`pwd -P` && cd -
if [ $? -ne 0 ]
then echo "cannot get this script's directory" 1>&2
     exit 1
fi

PKG_NAME=$(cd `git rev-parse --show-toplevel` && \
	       [ -f ./package.json ] && \
	       node -pe 'require("./package.json").name')
if [ $? -ne 0 ]
then echo "cannot get npm package's name" 1>&2
     exit 1
fi
export ROBOT_TYPE="$PKG_NAME"
export ROBOT_UUID=MA100101000019005100xxx
echo python3 "$ThisDir"/MQTT_teleoperation_rl.py "$@"
exec python3 "$ThisDir"/MQTT_teleoperation_rl.py "$@"
