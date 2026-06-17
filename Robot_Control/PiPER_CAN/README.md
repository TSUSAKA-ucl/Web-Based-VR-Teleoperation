# CANインターフェースの名前を固定する

0. usb-canのvenderとtypeを調べる
   ```
   lsusb
   ```
1. OpenMoko, Inc. Geschwister Schneider CAN adapterのserialを調べる
   ```
   for can in /sys/class/net/can*; \
   do echo $can; udevadm info -a "$can" | grep '{serial}'; done
   ```
   onboard usbハブのserialも出るが無視する
2. udev設定ファイルを作成
   ```
   sudo ./99-agilex-piper-can.rules /etc/udev/rules.d/
   sudo vi /etc/udev/rules.d/99-agilex-piper-can.rules
   ```
   `vi`でも`nano`でも良いので[上記ファイル](./99-agilex-piper-can.rules)の
   serialを書き換える。必要に応じで`NAME`も書き換えて良い。
   venderとidはOpenMokoのものにしてあるがもし違うチップが搭載されていたら書き換える
3. 設定ファイルを再読込
   ```
   sudo udevadm control --reload-rules
   ```
4. usb-canを抜いて挿し直す。インターフェース名確認
   ```
   ip -br a
   ```
   1.と同じコマンドでシリアル番号確認
   
以上で、usb-canの名前が固定される
