#!/bin/bash

# DockerコンテナからのGUI接続を許可
xhost +local:docker
# UID,GID環境変数をセットする
export UID=$(id -u)
export GID=$(id -g)
# スクリプト終了（Ctrl+C等）時に自動で権限を剥奪
trap 'xhost -local:docker' EXIT

# Docker Compose で全サービスを立ち上げ
docker compose up
