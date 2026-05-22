import argparse
import os
import ssl
import paho.mqtt.client as mqtt

# 1. コマンドライン引数の設定
def add_common_opts(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "-H",
        "--host",
        type=str,
        default="localhost",
        help="MQTTブローカーのホスト名 (IPまたはFQDN) [デフォルト: localhost]",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=8333,
        help="MQTTブローカーのポート番号 [デフォルト: 8333]",
    )
    return parser


# 2. $HOME ディレクトリを動的に取得してCA証明書のパスを作成
_home_dir = os.path.expanduser("~")
ca_certs_path = os.path.join(_home_dir, ".local/share/mkcert/rootCA.pem")


# 3. MQTTクライアントのTLS設定
## client = paho.mqtt.client.Client(transport="websockets")
def configure_tls(client : mqtt.Client) -> None:
    if os.path.exists(ca_certs_path):
        client.tls_set(
            ca_certs=ca_certs_path,
            certfile=None,  # クライアント証明書を使わない場合は None
            keyfile=None,
            cert_reqs=ssl.CERT_REQUIRED,  # サーバー証明書の検証を必須にする
            tls_version=ssl.PROTOCOL_TLSv1_2,  # または ssl.PROTOCOL_TLS
        )
        client.tls_insecure_set(False)
    else:
        client.tls_set(cert_reqs=0)
        client.tls_insecure_set(True)
