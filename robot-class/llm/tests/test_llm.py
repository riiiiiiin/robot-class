import sys
import rclpy
from rclpy.node import Node

# 这里假设你的自定义 srv 包名为 service_define，并且已经被正确安装 / 在工作空间中
# srv 类型名为 StringForString，唯一字段为 `string data`
from service_define.srv import StringForString

class LLMClientNode(Node):
    def __init__(self, service_name):
        super().__init__('llm_test_client')
        self.client = self.create_client(StringForString, service_name)
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'service {service_name} not available after 5s')
        else:
            self.get_logger().info(f'service {service_name} available')

    def call_extract_info(self, text: str):
        req = StringForString.Request()
        req.data = text

        self.get_logger().info(f'Sending request: "{text}"')
        future = self.client.call_async(req)
        return future


def llm_loop(service_name ,test_texts, args=None):
    rclpy.init(args=args)

    node = LLMClientNode(service_name)

    for send_text in test_texts.split('\n'):
        try:
            future = node.call_extract_info(send_text)

            # 等待异步 future 完成（官方 tutorial 推荐的方式可以使用 spin_until_future_complete）
            # 该调用会让 rclpy 运行直到 future 完成或超时。
            rclpy.spin_until_future_complete(node, future, timeout_sec=None)

            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    node.get_logger().error(f'Service call failed: {e}')
                else:
                    node.get_logger().info('Received response:')
                    print(response.data)
            else:
                node.get_logger().error('Service call did not complete (future not done).')

        finally:
            pass
    # 清理
    node.destroy_node()
    rclpy.shutdown()