#!/usr/bin/bash
cd `dirname "$0"`
python ./piper_v2_cmd.py -r "$@"
python ./piper_v2_cmd.py -l "$@"

