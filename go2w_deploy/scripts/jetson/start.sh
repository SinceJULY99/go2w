#!/bin/bash
# 用法: start.sh flat|dreamwaq [-y]
#   1. release_mode 关原厂运控(会先让狗趴下,并等你确认;-y 跳过确认)
#   2. 后台启动 go2w_ctrl,脱离 ssh 会话(断网线 / 断 ssh 不影响)。日志: ~/go2w_deploy/ctrl.log
# 两套策略的 policy_dir 和 FixStand 站姿不同,各存一份完整配置,启动时拷成 config.yaml。
POLICY=$1
CFG=~/go2w_deploy/deploy/robots/go2w/config
if [[ "$POLICY" != "flat" && "$POLICY" != "dreamwaq" ]]; then
  echo "用法: $0 flat|dreamwaq [-y]   (flat=平地策略, dreamwaq=爬台子策略)"; exit 1
fi
if pgrep -x go2w_ctrl >/dev/null; then echo "go2w_ctrl 已在运行 (pid $(pgrep -x go2w_ctrl)),先 stop.sh"; exit 1; fi
~/go2w_deploy/deploy/tools/release_mode/build/release_mode eth0 $2 || { echo "运控没关掉,不启动控制器"; exit 1; }
cp "$CFG/config.$POLICY.yaml" "$CFG/config.yaml"
cd ~/go2w_deploy/deploy/robots/go2w/build || exit 1
setsid nohup ./go2w_ctrl -n eth0 > ~/go2w_deploy/ctrl.log 2>&1 < /dev/null &
sleep 2
echo "[$POLICY] 已启动 pid $(pgrep -x go2w_ctrl)"; grep -E "Policy directory|SportMoveBridge|error|Error" ~/go2w_deploy/ctrl.log; tail -n 2 ~/go2w_deploy/ctrl.log
