#!/usr/bin/bash
GIT_TOPLEVEL=`git rev-parse --show-toplevel`
if [ $? -eq 0 ]; then
    cd "$GIT_TOPLEVEL"
    mkcert -cert-file own-ca-cert.pem -key-file own-ca-key.pem "$@"
fi
