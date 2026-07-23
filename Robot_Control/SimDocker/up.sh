#!/usr/bin/bash
xhost +local:`whoami`
Here=`pwd -P`
cd `dirname "$0"` && ThisDir=`pwd -P` && cd -
if [ $? -ne 0 ]
then echo "cannot get this script's directory" 1>&2
     exit 1
fi

if [ -f ~/.local/share/mkcert/rootCA.pem ] && \
       [ -f ../MQTT/rootCA.pem ] && \
       ! diff -q ~/.local/share/mkcert/rootCA.pem \
	 ../MQTT/rootCA.pem >/dev/null 2>&1
then cp -f ~/.local/share/mkcert/rootCA.pem ../MQTT/rootCA.pem
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
export MQTT_BROKER=`ip route get 8.8.8.8 | sed 's/^.*src \+\([0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+\) *.*$/\1/;t;d'`
docker compose up "$@"
