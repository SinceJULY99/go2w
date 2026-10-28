#!/bin/bash
# 停止前先用遥控器 LT+B 让狗趴下 —— 进程一退出电机就没指令了
pkill -INT -x go2w_ctrl && echo "已发送停止信号" || echo "go2w_ctrl 没在运行"
