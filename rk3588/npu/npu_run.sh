#!/usr/bin/env bash
# 免 root 使用新版 RKNN 运行时:
# 在用户命名空间里把本目录的 librknnrt.so 绑定挂载到 /usr/lib/librknnrt.so,
# 然后执行传入的命令。用法: ./npu_run.sh python3 your_script.py
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$1" = "--inner" ]; then
    shift
    LIB="${RKNN_LIB:-$HERE/librknnrt_250.so}"   # 默认 2.5.0, 兼容 1.6/2.x 模型
    mount --bind "$LIB" /usr/lib/librknnrt.so
    exec "$@"
fi

exec unshare -rm "$0" --inner "$@"
