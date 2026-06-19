# PiPERの`MQTT_teleoperation_rl.py`(本パッケージの`src/app/home.js`対応)におけるMetawork protocol用のtopic name

## このドキュメント位置づけ

commit ID `546d717170c2368c11b6a14b4666c9b0d50e6f39` 
(`deskmeet`ブランチ、Thu. Jun. 18 2026)は、
多少の不具合はあるが、運が良ければmanager[`MetaworkMQTT.py`](
./Mosquitto/MetaworkMQTT/MetaworkMQTT.py)を利用して左右とも動作する。
そこで使用されるトピック名は[`esa.io`のドキュメント](
https://uclab.esa.io/posts/9599) とは多少異なるため、本ドキュメントに書き出す。

今後、`MQTT_teleoperation_rl.py`をできるだけそのまま使って、ブラウザ側を
明示的なステートマシンに全面的に書き換え動作確認する。その後、複数ロボットの統合
の対応を、プロトコル(できるだけ`esa.io`ドキュメントに準拠する)・ブラウザ側・
エッジ(`MQTT_Client.py`)側で検討し、実装する。

## `esa.io`ドキュメントの各トピック名との対応

主に、[`MQTT/MQTT_Client.py`](./Robot_Control/MQTT/MQTT_Client.py)で
定義される。

## `mgr/register`
変更なし。[`MQTT_Client`のクラス変数でトピック名を定義](
https://github.com/TSUSAKA-ucl/Web-Based-VR-Teleoperation/blob/546d717170c2368c11b6a14b4666c9b0d50e6f39/Robot_Control/MQTT/MQTT_Client.py#L42) し、
ブローカーに接続直後[`on_connect`メソッド内でpublish](
https://github.com/TSUSAKA-ucl/Web-Based-VR-Teleoperation/blob/546d717170c2368c11b6a14b4666c9b0d50e6f39/Robot_Control/MQTT/MQTT_Client.py#L112) される。

## `dev/robot-id` 重要
* topic: `dev/+`
* callback: `on_recv_topic`

このcommit IDのフロントエンド側は`dev/robot-id`をpublishしないため、
`dev/`をwild cardでsubscribeして、payloadの`devId`が`self.ROBOT_UUID`
(すなわち`robot-id`)と等しいことを確認して、`split('/')`した`topic_name[1]`
から`user-id`をとりだす。

ただし、このコミットのコードのフロントエンドでは`robot/robot-id`のsubscribe宣言が
完了していない(間に合っていない)可能性がある。元々のエッジ側のコード
(`MQTT_teleoperation_right.py`など)では、はじめに1回だけ`robot/robot-id`を
publishし、そのままブラウザが準備完了になる(`control/user-id`のsubscriberが
設定するジョイント値が、`robot/robot-id`でpublishしたものと一致する)のを待つ。
ブラウザが1回目を取り落しているとデッドロックになる。

そのため、ブラウザの準備完了待ちのループ内でも数秒に1回`robot/robot-id`を
publishするように変更

## `robot/robot-id`
変更なし。エッジ(`MQTT_teleoperation*.py`)はtime,state,model,
アームのジョイント値をpayloadに乗せている。左右はペイロードで区別。
右と左でstateのための**キーが異なる**。右は`"state"`、左は`"state_left"`。
ペイロードに`"state"`キーが存在すれば右。ペイロードに`"state_left"`キーが
存在すれば左。ペイロードの他のキーも左右で別名。

## `control/user-id`
左右のアーム用に別々のトピック名でsubscribeしている。ただし固有のIDではない。
ロボットのUUIDは左右で同じ。ロボットのUUIDはトピック名に乗っていない。

右アーム用`MQTT_teleoperation_rl.py`
* `control/right/joint/[user-id]`
* `control/right/joint/[user-id]`
* `share/[user-id]` : ← 使用していないように見える

左アーム用`MQTT_teleoperation_rl.py`
* `control/left/joint/[user-id]`
* `control/left/joint/[user-id]`
* `share/[user-id]` : ← 使用していないように見える






# PiPERの一台分の`MQTT_teleoperation_rl.py`(現commit ID)の概念的状態遷移

0. dormant:
1. init:  
   `ROBOT_TYPE`と`ROBOT_UUID`は左右のアームで同一のものに
   設定(`MQTT_teleoperation_rl.sh`)  
   PiPER can接続  
   MQTT接続
   * `MQTT_Client`constructor:  
	 `mqtt.Client(arm, mode, args)`生成。`arm`:左右、`mode`:未使用、
	 `args`:broker IP,port,protocol,robot_type,robot_uuid
   * `start_mqtt`method:  
     configure TLS, `on_connect`&`on_disconnect`&`on_message`callback登録  
	 `connect(...)`, `loop_start()`
   
   PiPERから初期ジョイント値読み取り
2. ブラウザVR準備完了待ち:  
3. 受信目標値追従中:  
   * topicの最新メッセージのジョイント値取り出し
   * piperのジョイント値とりだし
   * 目標値との差分のRMSEが大きければ補間(217行 `for`)  
	 補間中はメッセージ(指令値)無視
4. 終了:  
   `robot/robot-id`にstateキーだけのペイロードを載せてpublish
