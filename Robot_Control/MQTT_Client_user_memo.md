## 旧PiPER版clientオブジェクト

### caller
`MQTT_teleoperation`から`MQTT_Client`内を呼び出している、
主に`paho.mqtt`に関係の深い部分を調査

1. constructor  
   変数定数定義のみ
2. `create_shared_memory` : 使っていない  
   `self.sm`と`self.pose`(view)を定義
3. `start_mqtt`  
   `connect`! `loop_start`!
4. `publish_message`(l.74)  
   publish `robot_state_message`
5. `set_ping_init`  
   set `self.ping`

基本的に以上。

### callee
旧`MQTT_Client`内の構造
1. constructor
2. `on_connect`  
   subscriber設定。  
   publish `mgr/register`
3. `on_disconnect`
4. `on_message`
   topic:`MQTT_CTRL_JOINT_TOPIC`と`MQTT_CTRL_TOOL_TOPIC`と`MQTT_SHARE_TOPIC`と解析
5. `start_mqtt`  
   (`on_connet`, `on_disconnect`, `on_message`)callback登録。
   connect, `loop_start()`

その他はshared memory関係と時間表示関係

## Unitree G1版clientオブジェクト

### caller
Unitree版では コンストラクターのcallerも(`main`も)`MQTT_Client.py`
1. constructor
2. `create_shared_memories`今回は気にしない
3. `connect_mqtt`
4. `publish_robot_state`  
   地の`while True:`内で1秒おきに呼ばれる
5. `robot_unregister`
   `is_connected`
   `disconnect`(optional)
   `loop_stop`
   `close_all_shm`  
   以上が`finally`でこの順で呼ばれる

### callee

1. `__init__(self, MQTT_Mode)`  
   インスタンス変数初期化のみ
2. `on_connect(self, client, userdata, flags,reason_code,properties)`  
   `mgr/register`をpublish
   `dev/robot_uuid`のsubscribe宣言
3. `on_disconnect(self, client, userdata, flags, reason_code, properties)`  
   とくに無し。ttyにmessage出力だけ

4. `on_message(self, client, userdata, msg)`  
   `dev/robot_uuid(self.MQTT_RECV_TOPIC)`受信と
   `control/XXXX(self.MQTT_CTRL_TOPIC)`受信のcallback処理。
   `dev/robot_uuid`受信時に`"devId"`を`self.USER_UUID`にセットして
   `self.MQTT_CTRL_TOPIC`を書き換えそのsubscriptionを実行。
   `control/XXXX`受信時はペイロードを解析してそれなりの処理を実行。
   Unitreeの場合はペイロードに左右アームハンドとウェストがあるはずでそれらをshmに
   書き出し

5. `connect_mqtt(self)`  
   localとuclabで条件分岐しているが内容はほとんど同じ(transportが異なるだけ)なので
   uclabは省略
   1. paho.mqtt.client.Client()生成
   2. `on_connect`,`on_disconnect`,`on_message`callback設定
   3. connect実行
   4. `loop_start()`

6. `create_shared_memories(self)` 略
7. `update_shm_ctrl(self, name, target_data)` 略
8. `update_shm_robot(self, name, robot_data)` 略
9. `close_all_shm(self)` 略

10. `publish_robot_state(self)`  
	`f"{MQTT_ROBOT_STATE_TOPIC}/{ROBOT_UUID}"`を即座にpublish。
	playloadはJSON string。qos=0

11. `robot_unregister(self)`  
	`mgr/unregister`トピックを即座にpublish


## 考察
1. shared memoryの生成やハンドルが、同一クラスに入っていて、
   shmの準備ができるまでMQTTに動いてもらいたくないため、
   paho.mqtt.clientの(生成と)connectがconstructorと別メソッドに
   してcaller側(main)でコントロールしているのであろう。
   PiPERteleop版ではclientの生成はconstructor内で、Unitree版は
   `connect_mqtt`(`start_mqtt`相当)メソッド内でclient
   オブジェクトを生成。後者は`connect_mqtt`を呼ぶと`self.client`
   が上書きされ無駄にclientオブジェクトが生成されるのでイマイチだと思うが...

2. PiPER版では、MQTT_Clientクラスのself.pose(np.array(16))が事実上callerに
   データを渡す(MQTT受信の場合)場所になっている。publishはcallerからdict型の
   payloadを引数で得ている
   
3. PiPER版を、最低限の改造でMetaworkMQTT.py(MQTT Manager)対応にするには
   1. `on_connect`の書き換え
	  1. `mgr/register`のpublish追加
