#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import json
import os
import uuid
from datetime import datetime, date, timedelta
# PyQt5 GUI相关导入
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGridLayout, QScrollArea, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QThread, QTimer
from PyQt5.QtGui import QFont, QPalette, QColor

# ROS2通信相关导入
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ================= 全局配置常量 =================
# 农历日期映射
LUNAR_MAPPING = {
    1: "初一", 2: "初二", 3: "初三", 4: "初四", 5: "初五", 6: "初六",
    7: "初七", 8: "初八", 9: "初九", 10: "初十", 11: "十一", 12: "十二",
    13: "十三", 14: "十四", 15: "十五", 16: "十六", 17: "十七", 18: "十八",
    19: "十九", 20: "二十", 21: "廿一", 22: "廿二", 23: "廿三", 24: "廿四",
    25: "廿五", 26: "廿六", 27: "廿七", 28: "廿八", 29: "廿九", 30: "三十", 31: "卅一"
}

# 颜色配置
THEME_COLOR = "#D488EA"    # 默认主题色（紫色）
ADD_COLOR = "#4CAF50"      # 新增操作高亮色（绿色）
UPDATE_COLOR = "#FFC107"   # 修改操作高亮色（黄色）
DELETE_COLOR = "#F44336"   # 删除操作高亮色（红色）
GET_COLOR = "#2196F3"      # 查询操作高亮色（蓝色）

# 时间配置
HIGHLIGHT_DURATION = 10000  # 高亮持续时间(ms)
DELETE_DELAY = 2000         # 删除操作延迟执行时间(ms)
TEMP_MONTH_DISPLAY_DURATION = 10000  # 临时切换月份显示时长(ms)

# 文件/ROS配置
JSON_PATH = os.environ.get('SCHEDULE_PATH')          # 日程数据本地存储路径
ROS_TOPIC_NAME = "/schedule_manager/response"  # ROS2订阅话题名

# ================= ROS2订阅模块 =================
class ROS2SubscriberNode(Node):
    """ROS2订阅节点，接收日程更新消息"""
    def __init__(self, signal_emitter):
        super().__init__("calendar_display_node")
        self.signal_emitter = signal_emitter  # Qt信号发射器
        # 创建订阅器
        self.subscription = self.create_subscription(
            String,
            ROS_TOPIC_NAME,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"已订阅ROS2话题: {ROS_TOPIC_NAME}")

    def listener_callback(self, msg):
        """ROS2消息回调函数"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"收到日程更新消息: {data}")
            # 发出原始字典，UI线程自行决定是否刷新（基于 ok/op）
            self.signal_emitter.ros_msg_received.emit(data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析失败: {e}")
            self.signal_emitter.feedback.emit("error", "数据格式错误，更新失败")

class ROS2Thread(QThread):
    """ROS2线程（避免阻塞Qt主线程）"""
    def run(self):
        try:
            rclpy.init()
            self.node = ROS2SubscriberNode(self.parent().signal_emitter)
            rclpy.spin(self.node)  # 持续监听ROS2消息
        except Exception as e:
            print(f"ROS2线程异常：{e}")
        finally:
            # 资源清理
            if hasattr(self, 'node'):
                self.node.destroy_node()
            rclpy.shutdown()

# ================= Qt信号发射器 =================
class SignalEmitter(QObject):
    """Qt信号发射器，用于ROS2线程与主线程通信"""
    ros_msg_received = pyqtSignal(dict)  # 接收ROS2消息信号
    feedback = pyqtSignal(str, str)     # 反馈信息信号(type, msg)

# ================= 自定义UI组件 =================
class ScheduleItem(QWidget):
    """日程卡片组件"""
    def __init__(self, data, highlight_color=None):
        super().__init__()
        self.data = data  # 日程数据 (fields: id, description, place, start_time, end_time)
        self.highlight_color = highlight_color or THEME_COLOR
        
        # 获取显示字段
        desc = data.get("description", "")
        place = data.get("place", "待定")
        st = data.get("start_time") or ""
        et = data.get("end_time") or ""
        # 仅显示时间部分（HH:MM）以便界面简洁；如果没有则显示空
        def time_of_day(ts):
            if not ts:
                return ""
            try:
                return ts[11:16]  # "YYYY-MM-DD HH:MM:SS" -> "HH:MM"
            except Exception:
                return ts
        t_start = time_of_day(st)
        t_end = time_of_day(et)
        
        self.setStyleSheet(f"background-color: transparent; border-radius: 4px;")
        
        # 主布局
        layout = QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(15)
        
        # 左侧高亮色条
        self.color_bar = QFrame()
        self.color_bar.setFixedWidth(6)
        self.color_bar.setStyleSheet(f"""
            background-color: {self.highlight_color}; 
            border-radius: 3px;
        """)
        layout.addWidget(self.color_bar)
        
        # 信息区： description + place
        info_vbox = QVBoxLayout()
        info_vbox.setSpacing(4)
        
        self.desc_label = QLabel(desc)
        self.desc_label.setFont(QFont("WenQuanYi Micro Hei", 18, QFont.Bold))
        self.desc_label.setStyleSheet(f"""
            color: white;
            text-shadow: 0 0 8px {self.highlight_color}80;
        """)
        info_vbox.addWidget(self.desc_label)
        
        self.place_label = QLabel(place)
        self.place_label.setFont(QFont("WenQuanYi Micro Hei", 12))
        self.place_label.setStyleSheet(f"""
            color: #E0E0E0;
            text-shadow: 0 0 6px {self.highlight_color}60;
        """)
        info_vbox.addWidget(self.place_label)
        
        layout.addLayout(info_vbox, 1)
        
        # 时间显示区域
        time_vbox = QVBoxLayout()
        time_vbox.setAlignment(Qt.AlignRight)
        
        self.time1_label = QLabel(t_start)
        self.time1_label.setFont(QFont("WenQuanYi Micro Hei", 16))
        self.time1_label.setStyleSheet(f"""
            color: white;
            text-shadow: 0 0 8px {self.highlight_color}80;
        """)
        
        self.time2_label = QLabel(t_end)
        self.time2_label.setFont(QFont("WenQuanYi Micro Hei", 12))
        self.time2_label.setStyleSheet(f"""
            color: #E0E0E0;
            text-shadow: 0 0 6px {self.highlight_color}60;
        """)
        
        time_vbox.addWidget(self.time1_label)
        time_vbox.addWidget(self.time2_label)
        
        layout.addLayout(time_vbox)
        self.setLayout(layout)
    
    def set_highlight(self, color):
        """动态更新高亮颜色"""
        self.highlight_color = color
        # 更新样式
        self.color_bar.setStyleSheet(f"""
            background-color: {self.highlight_color}; 
            border-radius: 3px;
        """)
        self.desc_label.setStyleSheet(f"""
            color: white;
            text-shadow: 0 0 8px {self.highlight_color}80;
        """)
        self.place_label.setStyleSheet(f"""
            color: #E0E0E0;
            text-shadow: 0 0 6px {self.highlight_color}60;
        """)
        self.time1_label.setStyleSheet(f"""
            color: white;
            text-shadow: 0 0 8px {self.highlight_color}80;
        """)
        self.time2_label.setStyleSheet(f"""
            color: #E0E0E0;
            text-shadow: 0 0 6px {self.highlight_color}60;
        """)

class DayCell(QWidget):
    """日历单元格组件"""
    clicked = pyqtSignal(int)  # 点击信号(传递日期)
    
    def __init__(self, day_num, is_current_month=True):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.day_num = day_num          # 日期数字
        self.is_current_month = is_current_month  # 是否当前月
        self.is_selected = False        # 是否选中
        self.has_event = False          # 是否有日程
        
        # 布局设置
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(15, 5, 5, 5)
        self.layout.setSpacing(5)
        self.layout.setAlignment(Qt.AlignCenter)

        # 日期数字
        self.num_label = QLabel(str(day_num))
        self.num_label.setFixedSize(60, 60)
        self.num_label.setFont(QFont("WenQuanYi Micro Hei", 18))
        self.num_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        
        # 农历日期
        self.lunar_label = QLabel(LUNAR_MAPPING.get(day_num, ""))
        self.lunar_label.setAlignment(Qt.AlignCenter)
        self.lunar_label.setFont(QFont("WenQuanYi Micro Hei", 10))
        
        # 日程标记点
        self.dot = QLabel()
        self.dot.setFixedSize(8, 8)
        self.dot.setStyleSheet(f"background-color: {THEME_COLOR}; border-radius: 4px;")
        self.dot.setVisible(False)

        # 添加组件到布局
        self.layout.addWidget(self.num_label, 0, Qt.AlignCenter)
        self.layout.addWidget(self.lunar_label, 0, Qt.AlignCenter)
        self.layout.addWidget(self.dot, 0, Qt.AlignCenter)
        self.setLayout(self.layout)
        
        self.update_style()  # 初始化样式

    def set_event_status(self, has_event):
        """设置是否有日程"""
        self.has_event = has_event
        self.update_style()

    def set_selected(self, selected):
        """设置选中状态"""
        self.is_selected = selected
        self.update_style()

    def update_style(self):
        """更新单元格样式"""
        if self.is_selected:
            # 选中状态样式
            self.num_label.setStyleSheet("""
                background-color: white; 
                color: black; 
                border-radius: 30px; 
                font-weight: bold;
            """)
            self.lunar_label.setStyleSheet("color: white;")
            self.dot.setVisible(False)
        elif self.is_current_month:
            # 当前月未选中样式
            self.num_label.setStyleSheet("background-color: transparent; color: white;")
            self.lunar_label.setStyleSheet("color: #8E8E93;")
            self.dot.setVisible(self.has_event)
        else:
            # 非当前月样式
            self.num_label.setStyleSheet("background-color: transparent; color: #3A3A3C;")
            self.lunar_label.setStyleSheet("color: #3A3A3C;")
            self.dot.setVisible(False)

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        if self.is_current_month:
            self.clicked.emit(self.day_num)
        super().mousePressEvent(event)

# ================= 主窗口 =================
class CalendarUI(QWidget):
    """日历主窗口（订阅 response 决定何时 reload 文件）"""
    def __init__(self):
        super().__init__()
        self.signal_emitter = SignalEmitter()  # 信号发射器
        self.schedule_data = self.load_local_data()  # 加载本地日程数据
        self.day_widgets = {}  # 日历单元格缓存
        self.current_selected_day = None  # 当前选中日期
        
        # 高亮标记缓存 {日程ID: {"color": 颜色, "op": 操作类型, "start_time": 开始时间}}
        self.highlight_marks = {}
        # 高亮清理定时器
        self.highlight_timer = QTimer()
        self.highlight_timer.setSingleShot(False)
        self.highlight_timer.timeout.connect(self.clear_expired_highlights)
        
        # 月份管理
        self.original_month = datetime.now().strftime("%Y-%m")  # 原始月份
        self.current_display_month = self.original_month       # 当前显示月份
        # 临时切换月份恢复定时器
        self.temp_month_timer = QTimer()
        self.temp_month_timer.setSingleShot(True)
        self.temp_month_timer.timeout.connect(self.restore_original_month)

        # 窗口基础设置
        self.setWindowTitle("ROS 日历机器人")
        self.setStyleSheet("background-color: black;")
        self.init_ui()  # 初始化UI

        # 启动ROS2线程（延迟1秒避免阻塞UI）
        QTimer.singleShot(1000, self.start_ros_thread)

        # 绑定信号
        self.signal_emitter.ros_msg_received.connect(self.handle_ros_msg)
        self.signal_emitter.feedback.connect(self.show_feedback)

        # 初始化选中当前日期
        self.select_current_date()
        
        # 启动高亮清理定时器（1秒检查一次）
        self.highlight_timer.start(1000)

    def init_ui(self):
        """初始化UI布局"""
        # 主垂直布局
        main_vbox = QVBoxLayout(self)
        main_vbox.setContentsMargins(40, 40, 40, 40)

        # 主水平布局（日历+日程）
        main_layout = QHBoxLayout()
        main_layout.setSpacing(50)

        # ========== 左侧日历面板 ==========
        left_panel = QWidget()
        left_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.left_vbox = QVBoxLayout(left_panel)
        self.left_vbox.setAlignment(Qt.AlignTop)

        # 月份标题
        self.month_title = QLabel(self.format_month_title(self.current_display_month))
        self.month_title.setFont(QFont("WenQuanYi Micro Hei", 48, QFont.Bold))
        self.month_title.setStyleSheet("color: white;")
        self.left_vbox.addWidget(self.month_title)
        self.left_vbox.addSpacing(30)

        # 星期头部
        self.grid_layout = QGridLayout()
        self.grid_layout.setSpacing(10)
        weekdays = ["日", "一", "二", "三", "四", "五", "六"]
        for i, w in enumerate(weekdays):
            l = QLabel(w)
            l.setAlignment(Qt.AlignCenter)
            l.setStyleSheet("color: #636366; font-size: 18px; font-weight: bold;")
            self.grid_layout.addWidget(l, 0, i)

        # 初始化日历格子
        self.refresh_calendar_grid(self.current_display_month)
        self.left_vbox.addLayout(self.grid_layout)
        self.left_vbox.addStretch()

        # ========== 右侧日程面板 ==========
        right_panel = QWidget()
        right_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.right_layout = QVBoxLayout(right_panel)
        self.right_layout.setContentsMargins(0, 20, 0, 0)
        self.right_layout.setAlignment(Qt.AlignTop)

        # 日程滚动区域
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("border: none; background: transparent;")
        self.scroll_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.scroll_content = QWidget()
        self.scroll_content.setStyleSheet("background: transparent;")
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_layout.setSpacing(8)
        self.scroll_layout.setAlignment(Qt.AlignTop)
        
        self.scroll_area.setWidget(self.scroll_content)
        self.right_layout.addWidget(self.scroll_area)

        # 组装主布局
        main_layout.addWidget(left_panel, stretch=1)
        main_layout.addWidget(right_panel, stretch=1)
        main_vbox.addLayout(main_layout, stretch=1)

        self.setLayout(main_vbox)
        self.update()
        self.resize(1200, 800)  # 默认窗口大小

    def format_month_title(self, month_str):
        """格式化月份标题（YYYY年MM月）"""
        year, month = month_str.split("-")
        return f"{year}年{month}月"

    def refresh_calendar_grid(self, target_month):
        """刷新日历格子"""
        # 清空原有格子（保留星期头部）
        while self.grid_layout.count() > 7:
            item = self.grid_layout.takeAt(7)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        self.day_widgets.clear()

        # 解析目标月份
        year, month = map(int, target_month.split("-"))
        first_day = date(year, month, 1)  # 当月第一天
        # 计算当月第一天是星期几（0=周一，6=周日 → 转换为UI显示的星期）
        first_weekday = first_day.weekday()
        first_weekday_ui = first_weekday + 1 if first_weekday != 6 else 0

        # 计算当月总天数
        if month == 12:
            next_month = date(year+1, 1, 1)
        else:
            next_month = date(year, month+1, 1)
        last_day = next_month - timedelta(days=1)
        total_days = last_day.day

        # 生成日期格子
        row, col = 1, first_weekday_ui
        for day in range(1, total_days + 1):
            cell = DayCell(day)
            # 检查该日期是否有日程
            date_str = f"{target_month}-{day:02d}"
            if date_str in self.schedule_data:
                cell.set_event_status(True)
            # 绑定点击事件
            cell.clicked.connect(self.handle_date_click)
            self.day_widgets[day] = cell
            self.grid_layout.addWidget(cell, row, col)

            col += 1
            if col > 6:
                col = 0
                row += 1

    def switch_to_temp_month(self, target_month):
        """临时切换显示月份"""
        if target_month == self.current_display_month:
            return

        self.original_month = self.current_display_month
        self.current_display_month = target_month
        # 更新标题样式
        self.month_title.setText(self.format_month_title(target_month))
        self.month_title.setStyleSheet(f"color: {THEME_COLOR}; font-size: 48px; font-weight: bold;")
        # 刷新日历
        self.refresh_calendar_grid(target_month)
        self.refresh_calendar_ui()

        # 启动恢复定时器
        self.temp_month_timer.start(TEMP_MONTH_DISPLAY_DURATION)

    def restore_original_month(self):
        """恢复显示原始月份"""
        self.current_display_month = self.original_month
        self.month_title.setText(self.format_month_title(self.original_month))
        self.month_title.setStyleSheet("color: white; font-size: 48px; font-weight: bold;")
        self.refresh_calendar_grid(self.original_month)
        self.refresh_calendar_ui()
        self.select_current_date()

    def select_current_date(self):
        """选中当前系统日期"""
        now = datetime.now()
        target_month = now.strftime("%Y-%m")
        target_day = now.day

        self.current_display_month = target_month
        self.month_title.setText(self.format_month_title(target_month))
        self.refresh_calendar_grid(target_month)

        # 更新选中状态
        if target_day in self.day_widgets:
            if self.current_selected_day and self.current_selected_day in self.day_widgets:
                self.day_widgets[self.current_selected_day].set_selected(False)
            self.day_widgets[target_day].set_selected(True)
            self.current_selected_day = target_day
            self.update_schedule_list(target_day)
        self.refresh_calendar_ui()

    def select_target_date(self, target_date_str):
        """选中指定日期"""
        try:
            year, month, day = map(int, target_date_str.split("-"))
            target_month = f"{year}-{month:02d}"

            self.current_display_month = target_month
            self.month_title.setText(self.format_month_title(target_month))
            self.refresh_calendar_grid(target_month)

            # 更新选中状态
            if day in self.day_widgets:
                if self.current_selected_day and self.current_selected_day in self.day_widgets:
                    self.day_widgets[self.current_selected_day].set_selected(False)
                self.day_widgets[day].set_selected(True)
                self.current_selected_day = day
                self.update_schedule_list(day)
            self.refresh_calendar_ui()
        except Exception as e:
            print(f"选中目标日期失败：{e}")

    def mark_highlight(self, schedule_id, op_type):
        """标记日程高亮"""
        # 操作类型-颜色映射
        color_map = {
            "add": ADD_COLOR,
            "update": UPDATE_COLOR,
            "delete": DELETE_COLOR,
            "get": GET_COLOR
        }
        if op_type in color_map:
            # 记录高亮信息
            self.highlight_marks[schedule_id] = {
                "color": color_map[op_type],
                "op": op_type,
                "start_time": datetime.now()
            }
            # 刷新日程列表
            self.update_schedule_list(self.current_selected_day)

    def clear_expired_highlights(self):
        """清理过期的高亮标记"""
        now = datetime.now()
        expired_ids = []
        # 检查所有高亮标记
        for sid, mark in self.highlight_marks.items():
            # 超过指定时长则标记为过期
            if (now - mark["start_time"]).total_seconds() * 1000 > HIGHLIGHT_DURATION:
                expired_ids.append(sid)
        # 删除过期标记
        for sid in expired_ids:
            del self.highlight_marks[sid]
        # 刷新UI
        if expired_ids:
            self.update_schedule_list(self.current_selected_day)

    # Note: 本地操作处理函数保留但在订阅响应模式下通常不直接调用（UI 以 reload 文件为准）
    def handle_add(self, args):
        """（保留）本地新增处理：不作为订阅驱动的主要更新路径"""
        schedule = args.get("schedule")
        if not schedule:
            print("添加失败：无日程数据")
            return
        # 本例中 UI 的主更新来自 reload 文件；此函数仅提供本地快速插入（可选）
        st = schedule.get("start_time")
        date_key = st[:10] if st else "__undated__"
        if not schedule.get("id"):
            schedule["id"] = f"local-{uuid.uuid4().hex[:6]}"
        if date_key not in self.schedule_data:
            self.schedule_data[date_key] = []
        self.schedule_data[date_key].append(schedule)
        # 排序基于 start_time（若存在）
        self.schedule_data[date_key].sort(key=lambda x: x.get("start_time") or "")
        self.mark_highlight(schedule["id"], "add")

    def handle_delete(self, args):
        """（保留）本地删除处理：不作为订阅驱动的主要更新路径"""
        schedule_id = args.get("id")
        if not schedule_id:
            print("删除失败：无日程ID")
            return
        self.mark_highlight(schedule_id, "delete")
        def delete_delayed():
            deleted = False
            for date, items in list(self.schedule_data.items()):
                for i, item in enumerate(items):
                    if item.get("id") == schedule_id:
                        del items[i]
                        deleted = True
                        if not items:
                            del self.schedule_data[date]
                        break
                if deleted:
                    break
            if not deleted:
                print(f"删除失败：无ID为{schedule_id}的日程")
            self.refresh_calendar_ui()
            self.save_local_data()
        QTimer.singleShot(DELETE_DELAY, delete_delayed)

    def handle_update(self, args):
        """（保留）本地修改处理"""
        schedule_id = args.get("id")
        patch = args.get("patch", {})
        if not schedule_id or not patch:
            print("修改失败：ID或修改内容为空")
            return
        updated = False
        for date, items in list(self.schedule_data.items()):
            for item in items:
                if item.get("id") == schedule_id:
                    item.update(patch)
                    new_st = item.get("start_time")
                    if new_st:
                        new_date = new_st[:10]
                        if new_date != date:
                            items.remove(item)
                            if not items:
                                del self.schedule_data[date]
                            if new_date not in self.schedule_data:
                                self.schedule_data[new_date] = []
                            self.schedule_data[new_date].append(item)
                    updated = True
                    break
            if updated:
                break
        if not updated:
            print(f"修改失败：无ID为{schedule_id}的日程")
            return
        self.mark_highlight(schedule_id, "update")

    def handle_get(self, args):
        """（保留）本地查询处理"""
        schedule_id = args.get("id")
        if schedule_id:
            for date, items in self.schedule_data.items():
                for item in items:
                    if item.get("id") == schedule_id:
                        self.mark_highlight(schedule_id, "get")
                        if item.get("start_time"):
                            self.select_target_date(item["start_time"][:10])
                        return
            print(f"查询失败：无ID为{schedule_id}的日程")
        else:
            self.refresh_calendar_ui()

    def handle_ros_msg(self, msg):
        """
        处理 ROS2 message：仅在收到成功的 add/delete/update/get response 时，
        才从文件 reload 全量 schedules 并刷新 UI；并尽量从响应中或重新加载的数据中推断受影响的 id/日期以高亮/跳转。
        """
        ok = bool(msg.get("ok", False))
        op = msg.get("op")
        args = msg.get("args", {}) or {}
        data = msg.get("data", {}) or {}

        # 仅关心成功的 CRUD/query 响应
        if not ok or op not in ("add", "delete", "update", "get", "list"):
            # 不刷新 UI
            return

        # 尽量推断受影响的 schedule id（优先从 response.data, fallback 到 args）
        affected_id = None
        if op == "add":
            affected_id = data.get("id") or args.get("schedule", {}).get("id")
        elif op == "delete":
            affected_id = args.get("id") or data.get("id")
        else:
            affected_id = args.get("id") or data.get("id") or (data.get("schedule", {}) or {}).get("id")

        # 重新从文件全量加载 schedules（保证以文件为单一真源）
        self.schedule_data = self.load_local_data()

        # 如果能找到 affected_id 在新加载的数据中，则高亮并跳转到对应日期
        target_date = None
        if affected_id:
            for date_key, items in self.schedule_data.items():
                for item in items:
                    if item.get("id") == affected_id:
                        target_date = date_key
                        break
                if target_date:
                    break

        # 如果找到了目标日期则临时切换显示并选中该日
        if target_date and target_date != "__undated__":
            # target_date 格式 "YYYY-MM-DD"
            try:
                self.switch_to_temp_month(target_date[:7])
                self.select_target_date(target_date)
            except Exception:
                pass

        # 标记高亮（基于 op 和 affected_id）
        if affected_id:
            self.mark_highlight(affected_id, "update" if op == "update" else op)

        # 最后刷新 UI 并保存（UI 的主状态来自文件）
        self.refresh_calendar_ui()
        self.save_local_data()

    def update_schedule_list(self, day_num):
        """更新右侧日程列表"""
        # 清空原有内容
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        
        # 获取当前日期的日程
        date_str = f"{self.current_display_month}-{day_num:02d}"
        events = self.schedule_data.get(date_str, [])
        
        if not events:
            # 无日程提示
            lbl = QLabel("今天无日程")
            lbl.setFont(QFont("WenQuanYi Micro Hei", 16))
            lbl.setStyleSheet("color: #636366; margin-top: 40px;")
            lbl.setAlignment(Qt.AlignCenter)
            self.scroll_layout.addWidget(lbl)
        else:
            # 生成日程卡片
            for event in events:
                # 获取高亮颜色
                highlight_color = None
                sid = event.get("id")
                if sid in self.highlight_marks:
                    highlight_color = self.highlight_marks[sid]["color"]
                
                # 创建日程卡片
                item_widget = ScheduleItem(event, highlight_color)
                
                # 删除操作强制红色高亮
                if sid in self.highlight_marks and self.highlight_marks[sid]["op"] == "delete":
                    item_widget.set_highlight(DELETE_COLOR)
                
                self.scroll_layout.addWidget(item_widget)
                
                # 添加分割线
                line = QFrame()
                line.setFixedHeight(1)
                line.setStyleSheet("background-color: #2C2C2E;")
                self.scroll_layout.addWidget(line)
        
        # 添加强制拉伸（防止内容过少时居中）
        self.scroll_layout.addStretch()

    def refresh_calendar_ui(self):
        """刷新日历UI（更新日程标记）"""
        for day_num, cell in self.day_widgets.items():
            date_str = f"{self.current_display_month}-{day_num:02d}"
            # 更新单元格日程标记
            cell.set_event_status(date_str in self.schedule_data and len(self.schedule_data[date_str]) > 0)
        # 刷新日程列表
        if self.current_selected_day:
            self.update_schedule_list(self.current_selected_day)

    def load_local_data(self):
        """加载本地存储的日程数据"""
        if not os.path.exists(JSON_PATH):
            return {}
        try:
            with open(JSON_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                processed = {}
                for item in data.get("schedules", []):
                    # item expected keys: id, description, place, start_time, end_time
                    st = item.get("start_time")
                    if st:
                        date_key = st[:10]  # "YYYY-MM-DD"
                    else:
                        date_key = "__undated__"
                    if date_key not in processed:
                        processed[date_key] = []
                    processed[date_key].append({
                        "id": item.get("id"),
                        "description": item.get("description", ""),
                        "place": item.get("place", ""),
                        "start_time": item.get("start_time"),
                        "end_time": item.get("end_time")
                    })
                # 对每个日期按 start_time 排序（无 start_time 放在末尾）
                for dkey, items in processed.items():
                    items.sort(key=lambda x: x.get("start_time") or "9999")
                return processed
        except Exception as e:
            print(f"加载本地数据失败: {e}")
            return {}

    def save_local_data(self):
        """保存日程数据到本地"""
        try:
            # 转换为列表格式
            all_schedules = []
            for date, items in self.schedule_data.items():
                # 把 __undated__ 的 key 转换为没有 start_time 的条目
                for it in items:
                    all_schedules.append({
                        "id": it.get("id"),
                        "description": it.get("description", ""),
                        "place": it.get("place", ""),
                        "start_time": it.get("start_time"),
                        "end_time": it.get("end_time")
                    })
            with open(JSON_PATH, "w", encoding="utf-8") as f:
                json.dump({"schedules": all_schedules}, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"保存本地数据失败: {e}")

    def handle_date_click(self, day_num):
        """处理日期点击事件"""
        # 取消原有选中
        if self.current_selected_day and self.current_selected_day in self.day_widgets:
            self.day_widgets[self.current_selected_day].set_selected(False)
        # 设置新选中
        self.day_widgets[day_num].set_selected(True)
        self.current_selected_day = day_num
        # 更新日程列表
        self.update_schedule_list(day_num)

    def show_feedback(self, type_, msg):
        """显示反馈信息"""
        print(f"[{type_}] {msg}")

    def start_ros_thread(self):
        """启动ROS2线程"""
        self.ros_thread = ROS2Thread(self)
        self.ros_thread.start()
        print("ROS2线程已启动")

    def keyPressEvent(self, event):
        """键盘事件：Esc退出"""
        if event.key() == Qt.Key_Escape:
            self.close()

    def closeEvent(self, event):
        """窗口关闭事件（清理ROS2线程）"""
        if hasattr(self, 'ros_thread') and self.ros_thread.isRunning():
            rclpy.shutdown()
            self.ros_thread.quit()
            self.ros_thread.wait()
        event.accept()

# ================= 程序入口 =================
if __name__ == "__main__":
    # 创建Qt应用
    app = QApplication(sys.argv)
    # 设置全局字体
    font = QFont("WenQuanYi Micro Hei", 10)
    app.setFont(font)
    # 创建并显示主窗口
    window = CalendarUI()
    window.showFullScreen()  # 全屏显示（适配机器人屏幕）
    # 运行应用
    sys.exit(app.exec_())