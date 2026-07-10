#!/bin/bash
# DockerコンテナからのGUI接続を許可 (CoppeliaSim用)
xhost +local:docker
# スクリプト終了（Ctrl+C等）時に自動で権限を剥奪
trap 'xhost -local:docker' EXIT

# UID,GID環境変数をセットする
if [ "$UID" != `id -u` ]; then
    unset UID
    export UID=$(id -u)
fi
export GID=$(id -g)

# サーバー用のcertsを../localcerts/にコピーする
CertsDir=../localcerts
if [ ! -f "$CertsDir"/localhost.pem ] || [ ! -f "$CertsDir"/localhost-key.pem ]
then "$CertsDir"/copy-certs.sh
fi

# さらにcertをbroker用にコピーする
function cmp_and_copy() {
    if [ ! -f "$2" ]; then
	cp "$1" "$2"
    elif ! cmp -s "$1" "$2"; then
	echo ERROR "$2" exists but is different from "$1". 1>&2
	exit 1
    fi
}
CADir=../Robot_Control/MQTT
BrokerCerts=../Mosquitto/config/certs/
cmp_and_copy "$CertsDir"/localhost.pem "$BrokerCerts"/localcerts.pem
cmp_and_copy "$CertsDir"/localhost-key.pem "$BrokerCerts"/localcerts-key.pem
cmp_and_copy "$CADir"/rootCA.pem "$BrokerCerts"/rootCA.pem

# MQTT client(MetaworkMQTT.py)用にホストのIPを取り出す
# HOSTIP=`ip route get 8.8.8.8 | sed 's/^.*src \+\([0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+\) *.*$/\1/;t;d'`
# export HOSTIP
# MetaworkMQTT.pyは同じcomposeネットワーク内のMosquittoコンテナに接続するので、ホストIPは不要になった。

# Docker Compose で全サービスを立ち上げ
docker compose up
