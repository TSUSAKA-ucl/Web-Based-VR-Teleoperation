#!/usr/bin/bash
x="$0"
while [ -L "$x" ]; do x=`readlink "$x"`; done; cd `dirname "$x"`
ThisDir=`pwd -P`
export GIT_TOPLEVEL=`git rev-parse --show-toplevel`
CONF_FILE=`mktemp /tmp/mosquitto.XXXXXXXXXX.conf`
envsubst < ./mosquitto.conf.templ > "$CONF_FILE"
exec mosquitto -c "$CONF_FILE" -v "$@"
