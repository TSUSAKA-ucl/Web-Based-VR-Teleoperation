# `MQTT_Client.py`用のroot認証局

プライベートIPでの開発ように`mkcert`による独自CAを使用している。
配布を容易にするためこのディレクトリにそのCAの公開鍵を置く。
(色々なプライベートネットのserverの認証を作成したく
mkcertが使うCAのkeyは直接問い合わせてください)  
別のrootCAを使うときは

各ホスト用の鍵は、個別にscpで配布する。置き場所は自由だが、面倒なので
`${HOME}/.local/share/ssl/`の下とする。ただし、Next.jsのdevサーバーの
`--experimental-https`にどのように認識させるかは要検討。今の所
パッケージルートの`certificates/localhost{,-key}.pem`からの
symlinkにしておけば書き換えられることなく認識することは確認すみ。

サーバーホスト用の鍵は、ssh等で手動で正しいホストに配布することとする。

## keyやcertが何がなにかわからなくなったとき

* 何(DNS, IP)を認証しているか調べるとき
  ```
  openssl x509 -in my-server-cert.pem -text -noout | grep -A3 "Subject Alternative Name"
  ```
* 所定のroot CAでサインされていることを調べるとき  
  (このディレクトリの`rootCA.pem`を使用しています)
  ```
  openssl verify -CAfile ./Robot_Control/MQTT/rootCA.pem my-server-cert.pem
  ```
* 自分の持っているcertとkeyが正しく対応しているか調べるとき
  ```
  ( openssl x509 -noout -modulus -in my-cert.pem |sha256sum; openssl rsa -noout -modulus -in my-key.pem | sha256sum)
  ```
