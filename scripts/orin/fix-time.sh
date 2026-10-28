#!/usr/bin/env bash
# Orin 时间修复：时区 + 开机等待 NTP 同步后再启动 Docker 容器
# 用法：sudo bash fix-time.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "请用 sudo 运行" >&2
  exit 1
fi

echo "[1/4] 时区设为 Asia/Shanghai"
timedatectl set-timezone Asia/Shanghai

echo "[2/4] 增加国内 NTP 源（开机更快完成同步）"
mkdir -p /etc/chrony/sources.d
cat > /etc/chrony/sources.d/cn.sources <<'EOF'
server ntp.aliyun.com iburst
server ntp.tencent.com iburst
EOF
chronyc reload sources >/dev/null || systemctl restart chrony

echo "[3/4] 启用 chrony-wait：开机时 time-sync.target 等到真正同步（最多 90s，无网也会继续开机）"
mkdir -p /etc/systemd/system/chrony-wait.service.d
cat > /etc/systemd/system/chrony-wait.service.d/override.conf <<'EOF'
[Service]
TimeoutStartSec=90
EOF
# 注：docker 只用 Wants/After 关联 time-sync.target，chrony-wait 超时失败也不会阻止 docker 启动
systemctl enable chrony-wait.service

echo "[4/4] Docker 在时间同步之后再启动（自启容器不再拿到 1970 年时间）"
mkdir -p /etc/systemd/system/docker.service.d
cat > /etc/systemd/system/docker.service.d/wait-time-sync.conf <<'EOF'
[Unit]
Wants=time-sync.target
After=time-sync.target
EOF

systemctl daemon-reload
echo
timedatectl | grep -E "Local time|Time zone|synchronized"
systemctl is-enabled chrony-wait.service
systemctl show docker -p After | tr ' ' '\n' | grep time-sync
echo "完成。下次重启生效；当前运行中的容器不受影响。"
