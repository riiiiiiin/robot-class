#!/usr/bin/env python3
import json
import os
import tempfile
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleScheduleManager(Node):
    """日程管理器：支持加载、添加、删除、修改、展示日程，并提供提醒功能"""

    SCHEMA_VERSION = "1.1"

    def __init__(self):
        super().__init__("schedule_manager")

        # 配置数据文件路径（默认 ~/.ros/schedule_data.json）
        default_path = str(Path.home() / "robot-class" / "schedule" / "schedule_data.json")
        self.declare_parameter("data_file", default_path)
        self.data_file = self.get_parameter("data_file").get_parameter_value().string_value or default_path

        # 加载日程数据
        self.schedules = self.load_schedules()

        # ========= ROS2 调用接口（JSON over Topic）=========
        # request:  std_msgs/String(JSON)
        # response: std_msgs/String(JSON)
        self.request_topic = "~/request"
        self.response_topic = "~/response"
        self._resp_pub = self.create_publisher(String, self.response_topic, 10)
        self._req_sub = self.create_subscription(String, self.request_topic, self._on_request_msg, 10)
        # ================================================

        # 定时检查提醒（默认每 5 秒）
        self.declare_parameter("check_period_sec", 5.0)
        period = float(self.get_parameter("check_period_sec").value)
        self.create_timer(period, self.check_reminders)

        self.get_logger().info(
            f"日程管理器启动，共有 {len(self.schedules)} 个日程，data_file={self.data_file} "
            f"(rpc topics: {self.get_fully_qualified_name()}/request , {self.get_fully_qualified_name()}/response)"
        )

    # -----------------------------
    # ROS2 Topic-RPC 接口
    # -----------------------------

    def _on_request_msg(self, msg: String) -> None:
        """
        功能:
          订阅 JSON 命令并执行，然后把结果以 JSON 形式发布到 response topic。

        输入:
          msg.data: JSON 字符串
            通用格式：
              {
                "request_id": "可选，用于对应请求",
                "op": "now|list|get|add|delete|update",
                "args": {...}
              }

        输出:
          发布到 response topic 的 JSON 字符串：
            {
              "request_id": "...",
              "ok": true/false,
              "op": "...",
              "data": {...},     # ok=true
              "error": {...}     # ok=false
            }
        """
        request_id = None
        op = None
        try:
            payload = json.loads(msg.data) if msg.data else {}
            if not isinstance(payload, dict):
                raise ValueError("request payload must be a JSON object")

            request_id = payload.get("request_id")
            op = payload.get("op")
            args = payload.get("args", {})
            if args is None:
                args = {}
            if not isinstance(args, dict):
                raise ValueError("args must be a JSON object")

            ok, data_or_err = self._handle_rpc(op, args)
            resp = {
                "request_id": request_id,
                "ok": ok,
                "op": op,
            }
            if ok:
                resp["data"] = data_or_err
            else:
                resp["error"] = data_or_err

        except Exception as e:
            resp = {
                "request_id": request_id,
                "ok": False,
                "op": op,
                "error": {"code": "BAD_REQUEST", "message": str(e)},
            }

        out = String()
        out.data = json.dumps(resp, ensure_ascii=False)
        self._resp_pub.publish(out)

    def _handle_rpc(self, op: Optional[str], args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """
        功能:
          根据 op 分发到具体功能函数，统一返回结果，便于 UI/大模型调用。

        输入:
          op: 操作名
          args: 参数对象

        输出:
          (ok, payload)
            ok=True  -> payload 为 data
            ok=False -> payload 为 error {code,message,details?}
        """
        if not op:
            return False, {"code": "MISSING_OP", "message": "op is required"}

        if op == "now":
            now_dt = datetime.now().astimezone()
            return True, {
                "iso": now_dt.isoformat(),
                "epoch_ms": int(now_dt.timestamp() * 1000),
            }

        if op == "list":
            # 可扩展过滤：status/tags/time-range 等
            return True, {"schedules": self.list_schedules()}

        if op == "get":
            schedule_id = args.get("id")
            if not schedule_id:
                return False, {"code": "MISSING_ID", "message": "args.id is required"}
            s = self.get_schedule(schedule_id)
            if not s:
                return False, {"code": "NOT_FOUND", "message": f"schedule not found: {schedule_id}"}
            return True, {"schedule": s}

        if op == "add":
            schedule = args.get("schedule")
            if not isinstance(schedule, dict):
                return False, {"code": "BAD_ARGS", "message": "args.schedule must be an object"}
            new_id, conflicts = self.add_schedule(schedule)
            return True, {"id": new_id, "conflicts": conflicts}

        if op == "delete":
            schedule_id = args.get("id")
            if not schedule_id:
                return False, {"code": "MISSING_ID", "message": "args.id is required"}
            ok = self.delete_schedule(schedule_id)
            return True, {"deleted": ok}

        if op == "update":
            schedule_id = args.get("id")
            patch = args.get("patch")
            if not schedule_id:
                return False, {"code": "MISSING_ID", "message": "args.id is required"}
            if not isinstance(patch, dict):
                return False, {"code": "BAD_ARGS", "message": "args.patch must be an object"}
            ok = self.update_schedule(schedule_id, patch)
            return True, {"updated": ok}

        return False, {"code": "UNKNOWN_OP", "message": f"unknown op: {op}"}

    # -----------------------------
    # 基础功能：时间与校验
    # -----------------------------

    def _now_iso(self) -> str:
        """功能: 获取当前时间的 ISO 字符串。输入: 无。输出: str(ISO8601)。"""
        return datetime.now().astimezone().isoformat()

    def _parse_iso(self, s: str) -> datetime:
        """功能: 解析 ISO 字符串为 datetime。输入: s(str)。输出: datetime(带时区)。"""
        dt = datetime.fromisoformat(s)
        if dt.tzinfo is None:
            return dt.replace(tzinfo=datetime.now().astimezone().tzinfo)
        return dt

    def _ensure_parent_dir(self, path: str) -> None:
        """功能: 确保 path 的父目录存在。输入: path(str)。输出: None。"""
        Path(path).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)

    def _atomic_write_json(self, path: str, data: Dict[str, Any]) -> None:
        """功能: 原子写 JSON。输入: path(str), data(dict)。输出: None。"""
        self._ensure_parent_dir(path)
        path = str(Path(path).expanduser())
        d = Path(path).parent
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=str(d), delete=False) as tf:
            json.dump(data, tf, indent=2, ensure_ascii=False)
            tmp_name = tf.name
        os.replace(tmp_name, path)

    def _normalize_schedule(self, s: Dict[str, Any]) -> Dict[str, Any]:
        """
        功能:
          归一化日程字段，兼容旧数据；补全缺省字段；修正类型。

        输入:
          s: 日程 dict（可能缺字段）

        输出:
          归一化后的日程 dict（原地补全并返回）
        """
        if "id" not in s:
            s["id"] = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-auto"
        s.setdefault("title", "未命名日程")
        s.setdefault("description", "")
        s.setdefault("start_time", None)
        s.setdefault("end_time", None)
        s.setdefault("duration", 60)
        s.setdefault("needs_reminder", True)
        s.setdefault("reminder_minutes", [30])
        s.setdefault("reminder_message", "即将开始的日程提醒")
        s.setdefault("notes", "")
        s.setdefault("tags", [])
        s.setdefault("priority", "medium")
        s.setdefault("status", "pending")
        s.setdefault("created_at", self._now_iso())

        rs = s.get("reminder_state")
        if not isinstance(rs, dict):
            rs = {}
        rs.setdefault("fired_minutes", [])
        s["reminder_state"] = rs

        if s["reminder_minutes"] is None:
            s["reminder_minutes"] = []
        if not isinstance(s["reminder_minutes"], list):
            s["reminder_minutes"] = [s["reminder_minutes"]]

        if s["tags"] is None:
            s["tags"] = []
        if not isinstance(s["tags"], list):
            s["tags"] = [s["tags"]]

        return s

    # -----------------------------
    # 持久化功能
    # -----------------------------

    def load_schedules(self) -> List[Dict[str, Any]]:
        """
        功能:
          从 data_file 加载 schedules；如果文件不存在或读取失败，返回空列表。

        输入:
          无（使用 self.data_file）

        输出:
          List[Dict]: 日程列表（已 normalize）
        """
        path = str(Path(self.data_file).expanduser())
        try:
            if os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)

                schedules = data.get("schedules", [])
                if not isinstance(schedules, list):
                    schedules = []

                normalized = [self._normalize_schedule(s) for s in schedules if isinstance(s, dict)]
                if normalized != schedules:
                    self.schedules = normalized
                    self.save_schedules()
                return normalized

            # 文件不存在：创建空表（满足“没有就创建空表”）
            self.schedules = []
            self.save_schedules()
            return []

        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
            return []

    def save_schedules(self) -> None:
        """
        功能:
          保存 schedules 到 data_file。

        输入:
          无（使用 self.schedules）

        输出:
          None
        """
        try:
            data = {
                "schema_version": self.SCHEMA_VERSION,
                "updated_at": self._now_iso(),
                "schedules": self.schedules,
            }
            self._atomic_write_json(self.data_file, data)
        except Exception as e:
            self.get_logger().error(f"保存失败: {e}")

    # -----------------------------
    # 日程管理功能
    # -----------------------------

    def get_schedule(self, schedule_id: str) -> Optional[Dict[str, Any]]:
        """
        功能: 按 id 获取日程。
        输入: schedule_id(str)
        输出: schedule(dict) 或 None
        """
        for s in self.schedules:
            if s.get("id") == schedule_id:
                return s
        return None

    def add_schedule(self, schedule_data: Dict[str, Any]) -> Tuple[str, List[Dict[str, Any]]]:
        """
        功能:
          添加新日程；若与现有日程时间段重叠，标记 status=conflict，并记录冲突列表。

        输入:
          schedule_data(dict): 至少建议包含 start_time/end_time（ISO 字符串）

        输出:
          (schedule_id, conflicts)
            schedule_id(str): 新建日程 id
            conflicts(list): 冲突项摘要列表 [{id,title,start_time,end_time}, ...]
        """
        schedule_id = schedule_data.get("id") or f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{len(self.schedules)}"

        schedule = {
            "id": schedule_id,
            "title": schedule_data.get("title", "未命名日程"),
            "description": schedule_data.get("description", ""),
            "start_time": schedule_data.get("start_time"),
            "end_time": schedule_data.get("end_time"),
            "duration": schedule_data.get("duration", 60),
            "needs_reminder": schedule_data.get("needs_reminder", True),
            "reminder_minutes": schedule_data.get("reminder_minutes", [30]),
            "reminder_message": schedule_data.get("reminder_message", "即将开始的日程提醒"),
            "notes": schedule_data.get("notes", ""),
            "tags": schedule_data.get("tags", []),
            "priority": schedule_data.get("priority", "medium"),
            "status": schedule_data.get("status", "pending"),
            "created_at": self._now_iso(),
        }
        schedule = self._normalize_schedule(schedule)

        conflicts: List[Dict[str, Any]] = []
        for existing in self.schedules:
            if self._is_conflict(schedule, existing):
                conflicts.append(
                    {
                        "id": existing.get("id"),
                        "title": existing.get("title"),
                        "start_time": existing.get("start_time"),
                        "end_time": existing.get("end_time"),
                    }
                )

        if conflicts:
            schedule["status"] = "conflict"
            self.get_logger().warning(f"日程冲突: {schedule['title']} (id={schedule_id}) conflicts={len(conflicts)}")

        self.schedules.append(schedule)
        self.save_schedules()
        self.get_logger().info(f"已添加日程: {schedule['title']} ({schedule_id})")
        return schedule_id, conflicts

    def delete_schedule(self, schedule_id: str) -> bool:
        """
        功能: 删除指定 id 的日程。
        输入: schedule_id(str)
        输出: bool(是否删除成功)
        """
        initial_count = len(self.schedules)
        self.schedules = [s for s in self.schedules if s.get("id") != schedule_id]
        if len(self.schedules) < initial_count:
            self.save_schedules()
            self.get_logger().info(f"已删除日程: {schedule_id}")
            return True
        return False

    def update_schedule(self, schedule_id: str, updates: Dict[str, Any]) -> bool:
        """
        功能: 更新指定 id 的日程字段（patch）。
        输入: schedule_id(str), updates(dict)
        输出: bool(是否更新成功)
        """
        for i, schedule in enumerate(self.schedules):
            if schedule.get("id") == schedule_id:
                schedule.update(updates)
                schedule["updated_at"] = self._now_iso()
                schedule = self._normalize_schedule(schedule)
                self.schedules[i] = schedule
                self.save_schedules()
                self.get_logger().info(f"已更新日程: {schedule_id}")
                return True
        return False

    def _is_conflict(self, new_schedule: Dict[str, Any], existing_schedule: Dict[str, Any]) -> bool:
        """
        功能: 判断两日程时间段是否冲突（重叠）。
        输入: new_schedule(dict), existing_schedule(dict)
        输出: bool
        """
        if not new_schedule.get("start_time") or not new_schedule.get("end_time"):
            return False
        if not existing_schedule.get("start_time") or not existing_schedule.get("end_time"):
            return False

        new_start = self._parse_iso(new_schedule["start_time"])
        new_end = self._parse_iso(new_schedule["end_time"])
        existing_start = self._parse_iso(existing_schedule["start_time"])
        existing_end = self._parse_iso(existing_schedule["end_time"])
        return not (new_end <= existing_start or new_start >= existing_end)

    # -----------------------------
    # 提醒功能
    # -----------------------------

    def check_reminders(self) -> None:
        """
        功能:
          定时检查所有日程，若到达提醒时间则触发提醒，并去重（reminder_state.fired_minutes）。

        输入: 无
        输出: None
        """
        now = datetime.now().astimezone()
        dirty = False

        for schedule in self.schedules:
            try:
                schedule = self._normalize_schedule(schedule)

                if schedule.get("status") in ("completed", "cancelled") or not schedule.get("needs_reminder", True):
                    continue

                start_time = schedule.get("start_time")
                if not start_time:
                    continue

                schedule_time = self._parse_iso(start_time)
                reminders: List[int] = schedule.get("reminder_minutes", []) or []
                fired: List[int] = schedule.get("reminder_state", {}).get("fired_minutes", []) or []

                for minutes in reminders:
                    if minutes in fired:
                        continue

                    reminder_time = schedule_time - timedelta(minutes=int(minutes))
                    if now >= reminder_time:
                        self.send_reminder(schedule, int(minutes))
                        fired.append(int(minutes))
                        schedule["reminder_state"]["fired_minutes"] = fired
                        dirty = True

            except Exception as e:
                self.get_logger().error(f"检查提醒失败: {e}")

        if dirty:
            self.save_schedules()

    def send_reminder(self, schedule: Dict[str, Any], minutes_before: int) -> None:
        """
        功能: 发送提醒（当前实现为日志输出；你也可以改成发 topic/通知 UI）。
        输入: schedule(dict), minutes_before(int)
        输出: None
        """
        message = schedule.get(
            "reminder_message",
            f"⏰ 提醒：{minutes_before}分钟后有日程：{schedule.get('title', '')}",
        )
        self.get_logger().info(message)

    # -----------------------------
    # 展示功能
    # -----------------------------

    def list_schedules(self) -> List[Dict[str, Any]]:
        """
        功能: 返回所有日程（给 UI / LLM 读取）。
        输入: 无
        输出: List[Dict]
        """
        return self.schedules


def main(args=None):
    rclpy.init(args=args)
    node = SimpleScheduleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_schedules()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()