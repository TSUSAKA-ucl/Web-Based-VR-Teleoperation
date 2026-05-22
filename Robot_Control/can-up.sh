#!/usr/bin/bash
function canUp () {
    local RES
    local x
    RES=`ip -br a show "$1"` && \
	echo "$RES" && \
	x=($RES) && \
	[[ ${x[1]} != 'UP' ]] && \
	sudo ip link set "$1" type can bitrate 1000000 && \
	sudo ip link set "$1" up &&\
	ip -br a show "$1"
}
canUp "can0"
canUp "can1"
