#!/usr/bin/bash
if [ "$0" = "-bash" ]; then
    # デフォルト補完(-D)を利用してコマンド名がv2_both.shにマッチすればこの補完を呼び出す
    _existing_default_func=$(complete -p -D 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="-F") print $(i+1)}')
    _v2_both_universal_completion() {
	local v2_both="${COMP_WORDS[0]##*/}"
	if [ "$v2_both" = "v2_both.sh" ]; then
	    local v2cmd
	    v2cmd=(enable disable go_zero joint reset stop read_joint)
	    local cur prev opts
	    cur="${COMP_WORDS[COMP_CWORD]}"
	    prev="${COMP_WORDS[COMP_CWORD-1]}"
	    if [ ${COMP_CWORD} -eq 1 ]; then
		# v2cmd 配列の内容を候補として返す
		COMPREPLY=( $(compgen -W "${v2cmd[*]}" -- ${cur}) )
		return 0
	    else
		# 第2引数以降は通常のファイル名補完を行う
		compopt -o filenames
		COMPREPLY=( $(compgen -f -- ${cur}) )
		return 0
	    fi
	elif [ -n "$_existing_default_func" ]; then
	    # 既存のデフォルト補完があればそれを呼ぶ
	    "$_existing_default_func" "$@"
	else
	    # なければ通常のファイル補完
	    local cur="${COMP_WORDS[COMP_CWORD]}"
            COMPREPLY=( $(compgen -f -- "$cur") )
	fi
    }
    # complete -F _v2_both_completion v2_both.sh
    complete -F _v2_both_universal_completion -D
    return
fi
cd `dirname "$0"`
python ./piper_v2_cmd.py -r "$@"
python ./piper_v2_cmd.py -l "$@"
