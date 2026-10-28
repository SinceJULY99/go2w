#!/usr/bin/env bash
# 在 Mac 上运行：把仓库同步到导航主机的 ~/go2w
#   scripts/sync.sh          同步到 Orin（默认）
#   scripts/sync.sh dock     同步到扩展坞（PC2）
#   scripts/sync.sh all      两台都同步
# Orin 的 Wi-Fi 不通时自动经 PC2 走有线（PC2 -> 192.168.123.222）
set -euo pipefail
cd "$(dirname "$0")/.."

ORIN_WIFI=${ORIN_WIFI:-192.168.1.111}
ORIN_WIRED=${ORIN_WIRED:-192.168.123.222}
PC2=${PC2:-unitree@192.168.1.142}

sync_to() {  # sync_to <rsh> <dest>
  # third_party、编译产物、运行数据只存在于目标机器，不能被 --delete 删掉
  # go2w_deploy 是另一个项目，不随导航同步
  rsync -az --delete -e "$1" \
    --exclude .git/ --exclude .DS_Store --exclude __pycache__/ \
    --exclude third_party/ \
    --exclude ws/build/ --exclude ws/install/ --exclude ws/log/ \
    --exclude data/ --exclude go2w_deploy/ --exclude docker_build.log \
    ./ "$2"
  echo "已同步到 $2"
}

sync_orin() {
  if nc -z -G 3 "$ORIN_WIFI" 22 2>/dev/null || nc -z -w 3 "$ORIN_WIFI" 22 2>/dev/null; then
    sync_to "ssh" "w@${ORIN_WIFI}:go2w/"
  else
    echo "Orin Wi-Fi 不通，经 PC2 跳板同步"
    sync_to "ssh -o HostKeyAlias=${ORIN_WIFI} -J ${PC2}" "w@${ORIN_WIRED}:go2w/"
  fi
}

case "${1:-orin}" in
  orin) sync_orin ;;
  dock) sync_to "ssh" "${PC2}:go2w/" ;;
  all)  sync_orin; sync_to "ssh" "${PC2}:go2w/" ;;
  *)    sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
