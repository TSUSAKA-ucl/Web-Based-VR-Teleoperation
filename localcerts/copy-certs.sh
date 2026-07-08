#!/usr/bin/bash
# このホストのサーバー認証用のcertとkeyのコピーをここにつくる
ThisDir=$(cd $(dirname $0); pwd -P)
# 証明書のディレクトリ
CertDir="${HOME}/.local/share/ssl"
# $CertDirに *.pemと*-key.pemの2個があれば、そのリンクを
# (locahost専用でなくても)localhost.pemとlocalhost-key.pemに作る
#
cd "$CertDir"
KeyFileCount=$(ls -1 *-key.pem 2>/dev/null | wc -l)
if [ $KeyFileCount -eq 1 ]; then
    KeyFile=$(ls -1 *-key.pem)
    CertFileName=$(echo "$KeyFile" | sed 's/-key\.pem$/.pem/')
    if [ -f "$CertFileName" ]; then
	cd "$ThisDir"
	cp -p "$CertDir/$CertFileName" "$ThisDir/localhost.pem"
	cp -p "$CertDir/$KeyFile" "$ThisDir/localhost-key.pem"
	chmod 600 "$ThisDir/localhost.pem" "$ThisDir/localhost-key.pem"
	echo "Copy cert and key files to $ThisDir"
	exit 0
    else
	echo "Error: Corresponding certificate file $CertFileName not found for key $KeyFile"
    fi
else
    echo "Error: Expected exactly one key file in $CertDir, found $KeyFile
"
fi
exit 1
