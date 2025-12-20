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
    """日程管理器"""

    SCHEMA_VERSION = "1.1"
    TIME_FORMAT = "%Y-%m-%d %H:%M:%S"

    def __init__(self):
        super().__init__("schedule_manager")

        # 配置数据文件路径（默认 ~/.ros/schedule_data.json）
        default_path = str(Path.home() / "robot-class" / "schedule" / "schedule_data.json")
        self.declare_parameter("data_file", default_path)
        self.data_file = self.get_parameter("data_file").get_parameter_value().string_value or default_path

        # internal next id (auto-increment). Will be initialized in load_schedules().
        self._next_id: int = 0

        # 加载日程数据（同时设置 self._next_id）
        self.schedules = self.load_schedules()

        # ========= ROS2 调用接口（JSON over Topic）=========
        # request:  std_msgs/String(JSON)
        # response: std_msgs/String(JSON)
        self.request_topic = "~/request"
        self.response_topic = "~/response"
        self._resp_pub = self.create_publisher(String, self.response_topic, 10)
        self._req_sub = self.create_subscription(String, self.request_topic, self._on_request_msg, 10)
        # ================================================

        # 注意：已移除提醒相关的定时器/方法

        self.get_logger().info(
            f"日程管理器启动，共有 {len(self.schedules)} 个日程，next_id={self._next_id}, data_file={self.data_file} "
            f"(rpc topics: {self.get_fully_qualified_name()}/request , {self.get_fully_qualified_name()}/response)"
        )

    # -----------------------------
    # ROS2 Topic-RPC 接口
    # -----------------------------

    def _on_request_msg(self, msg: String) -> None:
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
    # 基础功能：时间解析与格式化
    # -----------------------------

    def _now_str(self) -> str:
        """返回本地时区的当前时间字符串，格式 TIME_FORMAT。"""
        return datetime.now().astimezone().strftime(self.TIME_FORMAT)

    def _parse_time_to_dt(self, s: str) -> datetime:
        """
        解析时间字符串到带本地时区的 datetime。
        支持：
          - ISO 格式（datetime.fromisoformat）
          - "YYYY-MM-DD HH:MM:SS"（TIME_FORMAT）
        解析后若无 tzinfo，会设置为本地时区。
        """
        if s is None:
            raise ValueError("time string is None")
        if isinstance(s, datetime):
            dt = s
        else:
            # 尝试 ISO
            try:
                dt = datetime.fromisoformat(s)
            except Exception:
                # 再尝试自定义格式
                try:
                    dt = datetime.strptime(s, self.TIME_FORMAT)
                except Exception as e:
                    raise ValueError(f"无法解析时间字符串: {s}") from e

        if dt.tzinfo is None:
            return dt.replace(tzinfo=datetime.now().astimezone().tzinfo)
        return dt

    def _format_dt(self, dt: datetime) -> str:
        """格式化 datetime 为 TIME_FORMAT 字符串（本地时区）。"""
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=datetime.now().astimezone().tzinfo)
        return dt.astimezone().strftime(self.TIME_FORMAT)

    def _ensure_parent_dir(self, path: str) -> None:
        Path(path).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)

    def _atomic_write_json(self, path: str, data: Dict[str, Any]) -> None:
        self._ensure_parent_dir(path)
        path = str(Path(path).expanduser())
        d = Path(path).parent
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=str(d), delete=False) as tf:
            json.dump(data, tf, indent=2, ensure_ascii=False)
            tmp_name = tf.name
        os.replace(tmp_name, path)

    def _generate_id(self) -> str:
        """返回下一个自增 id（字符串），并递增内部计数。"""
        id_str = str(self._next_id)
        self._next_id += 1
        return id_str

    def _normalize_schedule(self, s: Dict[str, Any]) -> Dict[str, Any]:
        """
        归一化日程字段，仅保留 id, description, place, start_time, end_time。
        若 id 缺失则分配自增 id；若 start_time/end_time 可解析则标准化为 TIME_FORMAT 字符串。
        其它字段会被移除。
        """
        normalized: Dict[str, Any] = {}

        # id
        if "id" not in s or s.get("id") is None or str(s.get("id")) == "":
            normalized["id"] = self._generate_id()
        else:
            normalized["id"] = str(s.get("id"))

        # description 优先使用传入的 description；如果没有则用空字符串
        normalized["description"] = s.get("description") or ""

        # place
        normalized["place"] = s.get("place") or ""

        # start_time / end_time：尝试解析并标准化为 TIME_FORMAT 字符串；解析失败则保留原字符串
        st = s.get("start_time")
        et = s.get("end_time")
        try:
            if st:
                dt = self._parse_time_to_dt(st)
                normalized["start_time"] = self._format_dt(dt)
            else:
                normalized["start_time"] = None
        except Exception:
            normalized["start_time"] = st

        try:
            if et:
                dt = self._parse_time_to_dt(et)
                normalized["end_time"] = self._format_dt(dt)
            else:
                normalized["end_time"] = None
        except Exception:
            normalized["end_time"] = et

        return normalized

    # -----------------------------
    # 持久化功能
    # -----------------------------

    def load_schedules(self) -> List[Dict[str, Any]]:
        path = str(Path(self.data_file).expanduser())
        try:
            if os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)

                schedules = data.get("schedules", [])
                if not isinstance(schedules, list):
                    schedules = []

                # 找出文件中已有的最大 numeric id，用于设置 self._next_id
                max_id = -1
                for s in schedules:
                    try:
                        sid = s.get("id")
                        if sid is None:
                            continue
                        sid_int = int(sid)
                        if sid_int > max_id:
                            max_id = sid_int
                    except Exception:
                        # 非数字 id 跳过
                        continue
                self._next_id = max_id + 1 if max_id >= 0 else 0

                # 归一化所有条目（只保留新的字段集）
                normalized = [self._normalize_schedule(dict(s)) for s in schedules if isinstance(s, dict)]

                # 如果归一化后修改了内容，保存回文件
                if normalized != schedules:
                    self.schedules = normalized
                    self.save_schedules()
                return normalized

            # 文件不存在：创建空表并保存（_next_id 默认为 0）
            self.schedules = []
            self._next_id = 0
            self.save_schedules()
            return []

        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
            self._next_id = 0
            return []

    def save_schedules(self) -> None:
        try:
            data = {
                "schema_version": self.SCHEMA_VERSION,
                "updated_at": self._now_str(),
                "schedules": self.schedules,
            }
            self._atomic_write_json(self.data_file, data)
        except Exception as e:
            self.get_logger().error(f"保存失败: {e}")

    # -----------------------------
    # 日程管理功能
    # -----------------------------

    def get_schedule(self, schedule_id: str) -> Optional[Dict[str, Any]]:
        for s in self.schedules:
            if s.get("id") == str(schedule_id):
                return s
        return None

    def add_schedule(self, schedule_data: Dict[str, Any]) -> Tuple[str, List[Dict[str, Any]]]:
        """
        添加新日程；仅接受并保存 description/place/start_time/end_time；返回 (new_id, conflicts)
        新 id 使用全局自增，不回收删除编号。
        """
        incoming_id = schedule_data.get("id")
        if incoming_id is not None:
            incoming_id = str(incoming_id)
        if incoming_id is None or incoming_id == "":
            schedule_id = self._generate_id()
        else:
            try:
                n = int(incoming_id)
                schedule_id = str(n)
                if n >= self._next_id:
                    self._next_id = n + 1
            except Exception:
                schedule_id = incoming_id

        schedule = {
            "id": schedule_id,
            "description": schedule_data.get("description", ""),
            "place": schedule_data.get("place", ""),
            "start_time": schedule_data.get("start_time"),
            "end_time": schedule_data.get("end_time"),
        }
        schedule = self._normalize_schedule(schedule)

        conflicts: List[Dict[str, Any]] = []
        for existing in self.schedules:
            if self._is_conflict(schedule, existing):
                conflicts.append(
                    {
                        "id": existing.get("id"),
                        "description": existing.get("description"),
                        "start_time": existing.get("start_time"),
                        "end_time": existing.get("end_time"),
                    }
                )

        if conflicts:
            self.get_logger().warning(f"日程冲突: id={schedule_id} conflicts={len(conflicts)}")

        self.schedules.append(schedule)
        self.save_schedules()
        self.get_logger().info(f"已添加日程: {schedule_id}")
        return schedule_id, conflicts

    def delete_schedule(self, schedule_id: str) -> bool:
        initial_count = len(self.schedules)
        self.schedules = [s for s in self.schedules if s.get("id") != str(schedule_id)]
        if len(self.schedules) < initial_count:
            self.save_schedules()
            self.get_logger().info(f"已删除日程: {schedule_id}")
            return True
        return False

    def update_schedule(self, schedule_id: str, updates: Dict[str, Any]) -> bool:
        """
        更新仅允许修改 description/place/start_time/end_time；其它键将被忽略。
        """
        for i, schedule in enumerate(self.schedules):
            if schedule.get("id") == str(schedule_id):
                # 只保留允许的字段
                allowed = {}
                if "description" in updates:
                    allowed["description"] = updates.get("description")
                if "place" in updates:
                    allowed["place"] = updates.get("place")
                if "start_time" in updates:
                    allowed["start_time"] = updates.get("start_time")
                if "end_time" in updates:
                    allowed["end_time"] = updates.get("end_time")

                # 应用并归一化
                schedule.update(allowed)
                schedule = self._normalize_schedule(schedule)
                self.schedules[i] = schedule
                self.save_schedules()
                self.get_logger().info(f"已更新日程: {schedule_id}")
                return True
        return False

    def _is_conflict(self, new_schedule: Dict[str, Any], existing_schedule: Dict[str, Any]) -> bool:
        """
        判断两日程时间段是否冲突（重叠）。
        时间字符串采用 TIME_FORMAT（也兼容 ISO）进行解析比较。
        """
        if not new_schedule.get("start_time") or not new_schedule.get("end_time"):
            return False
        if not existing_schedule.get("start_time") or not existing_schedule.get("end_time"):
            return False

        try:
            new_start = self._parse_time_to_dt(new_schedule["start_time"])
            new_end = self._parse_time_to_dt(new_schedule["end_time"])
            existing_start = self._parse_time_to_dt(existing_schedule["start_time"])
            existing_end = self._parse_time_to_dt(existing_schedule["end_time"])
        except Exception:
            # 解析错误时保守处理为不冲突
            return False

        return not (new_end <= existing_start or new_start >= existing_end)

    # -----------------------------
    # 展示功能
    # -----------------------------

    def list_schedules(self) -> List[Dict[str, Any]]:
        # 返回当前列表（每项仅包含 id, description, place, start_time, end_time）
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