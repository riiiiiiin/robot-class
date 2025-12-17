#!/usr/bin/env bash
# filepath: /home/schuyler/robot-class/schedule/demo_schedule_rpc.sh
set -euo pipefail

# ROS 2 setup 脚本在 nounset(-u) 下可能引用未定义变量，需临时关闭 -u
set +u
source /opt/ros/jazzy/setup.bash
set -u

REQ_TOPIC="/schedule_manager/request"

RID=100
call() {
  local json="$1"
  ros2 topic pub --once "${REQ_TOPIC}" std_msgs/msg/String "{data: '${json}'}"
  sleep 0.2
}

# ...existing code...
TS="$(date +%s)"
ID1="demo-${TS}-event-1"
ID2="demo-${TS}-event-2-conflict"

T1_START="2099-01-01T10:00:00+08:00"
T1_END="2099-01-01T11:00:00+08:00"
T2_START="2099-01-01T10:30:00+08:00"
T2_END="2099-01-01T11:30:00+08:00"

echo "[1] now"
call "{\"request_id\":\"$((RID++))\",\"op\":\"now\",\"args\":{}}"

echo "[2] list (initial)"
call "{\"request_id\":\"$((RID++))\",\"op\":\"list\",\"args\":{}}"

echo "[3] add (non-conflict) id=${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"add\",\"args\":{\"schedule\":{\"id\":\"${ID1}\",\"title\":\"Demo事件1\",\"description\":\"用于测试 add/get/update/delete\",\"start_time\":\"${T1_START}\",\"end_time\":\"${T1_END}\",\"needs_reminder\":true,\"reminder_minutes\":[30,10],\"reminder_message\":\"Demo提醒：事件1快开始了\",\"notes\":\"初始备注\"}}}"

echo "[4] get id=${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"get\",\"args\":{\"id\":\"${ID1}\"}}"

echo "[5] update (patch) id=${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"update\",\"args\":{\"id\":\"${ID1}\",\"patch\":{\"title\":\"Demo事件1(已更新)\",\"notes\":\"更新后的备注\",\"needs_reminder\":false}}}"

echo "[6] get (after update) id=${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"get\",\"args\":{\"id\":\"${ID1}\"}}"

echo "[7] add (conflict) id=${ID2} overlaps with ${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"add\",\"args\":{\"schedule\":{\"id\":\"${ID2}\",\"title\":\"Demo事件2(冲突)\",\"description\":\"这个事件与事件1时间重叠\",\"start_time\":\"${T2_START}\",\"end_time\":\"${T2_END}\",\"needs_reminder\":false,\"reminder_minutes\":[15],\"reminder_message\":\"你不该看到这条（needs_reminder=false）\",\"notes\":\"用于冲突测试\"}}}"

echo "[8] list (should include conflict status on event2)"
call "{\"request_id\":\"$((RID++))\",\"op\":\"list\",\"args\":{}}"

echo "[9] delete id=${ID1}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"delete\",\"args\":{\"id\":\"${ID1}\"}}"

echo "[10] delete id=${ID2}"
call "{\"request_id\":\"$((RID++))\",\"op\":\"delete\",\"args\":{\"id\":\"${ID2}\"}}"

echo "[11] list (final)"
call "{\"request_id\":\"$((RID++))\",\"op\":\"list\",\"args\":{}}"

echo "Done. Check terminal-2 output for responses."