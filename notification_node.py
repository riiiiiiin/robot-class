import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from service_define.srv import SetString 
import json
import time
import uuid
from datetime import datetime, timedelta

class NotificationNode(Node):
    def __init__(self):
        super().__init__('notification_node')
        
        # 1. 日程管理器接口 (Topic RPC)
        self.editor_pub = self.create_publisher(String, '/schedule_manager/request', 10)
        self.editor_sub = self.create_subscription(String, '/schedule_manager/response', self.handle_schedule_response, 10)
        
        # 2. TTS 服务客户端 (注意：这里使用 Service 而不是 Topic)
        self.tts_client = self.create_client(SetString, 'tts_service')
        
        # 等待 TTS 服务上线（非阻塞等待，避免卡死初始化）
        self.get_logger().info('Waiting for TTS service...')
        
        # 3. 定时器：每 30 秒检查一次日程
        self.check_timer = self.create_timer(30.0, self.trigger_check)
        
        # 缓存：记录已经提醒过的日程ID，防止重复唠叨
        self.notified_events = set()
        
        # 配置：提前 N 分钟提醒
        self.reminder_minutes = 10 
        
        self.current_request_id = None
        self.get_logger().info(f'Notification Node started. Reminding {self.reminder_minutes} mins ahead.')

    def trigger_check(self):
        """发送获取日程列表的请求"""
        self.current_request_id = f"notify-{uuid.uuid4().hex[:8]}"
        req = {
            "request_id": self.current_request_id,
            "op": "list",
            "args": {}
        }
        msg = String()
        msg.data = json.dumps(req)
        self.editor_pub.publish(msg)

    def handle_schedule_response(self, msg):
        """处理日程管理器的返回数据"""
        try:
            resp = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # 过滤掉不是本节点发起的请求
        if resp.get('request_id') != self.current_request_id:
            return
            
        if not resp.get('ok'):
            return

        schedules = resp.get('data', {}).get('schedules', [])
        self.check_upcoming_events(schedules)

    def check_upcoming_events(self, schedules):
        """检查是否有即将开始的日程"""
        now = datetime.now()
        
        for schedule in schedules:
            sid = schedule.get('id')
            start_str = schedule.get('start_time', '')
            description = schedule.get('description', '未命名事项')
            
            if not start_str or sid in self.notified_events:
                continue

            try:
                clean_time_str = start_str.replace('T', ' ')
                if len(clean_time_str) > 19: 
                    clean_time_str = clean_time_str[:19]
                    
                start_time = datetime.strptime(clean_time_str, "%Y-%m-%d %H:%M:%S")
                
                # 计算时间差
                diff = start_time - now
                
                # 逻辑：在 (0, reminder_minutes] 区间内触发提醒
                if timedelta(seconds=0) < diff <= timedelta(minutes=self.reminder_minutes):
                    minutes_left = int(diff.total_seconds() / 60) + 1
                    
                    # 组织语音文本
                    tts_text = f"温馨提醒：{minutes_left}分钟后，您有日程：{description}。"
                    
                    self.speak(tts_text)
                    
                    # 标记为已提醒
                    self.notified_events.add(sid)
                    self.get_logger().info(f'Notified event: {description} at {start_str}')
                    
            except ValueError as e:
                self.get_logger().warn(f"Time parse failed for {sid}: {e}")

    def speak(self, text):
        """调用 TTS 服务发声"""
        if not self.tts_client.service_is_ready():
            self.get_logger().warn('TTS service not ready, skipping notification.')
            return
            
        req = SetString.Request()
        req.data = text
        # 异步调用，不阻塞定时器
        self.tts_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = NotificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()