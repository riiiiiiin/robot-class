import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Dict, Any
import json
import uuid
from enum import Enum

from service_define.srv import StringForString
from service_define.srv import SetString

def make_request_id() -> str:
    """Generate a compact request id: <timestamp_ms>-<shortuuid>"""
    ts_ms = int(time.time() * 1000)
    short = uuid.uuid4().hex[:8]
    return f"{ts_ms}-{short}"

class HubNode(Node):
    class State(Enum):
        IDLE = 0,
        NEED_CONFIRM = 1,
    def __init__(self, node_name: str = 'hub_node'):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter('asr_topic', '/asr_result')
        self.declare_parameter('editor_topic', '/schedule_manager/request')
        self.declare_parameter('llm_chat_service', 'llm/chat')
        self.declare_parameter('llm_generate_service', 'llm/generate_operation')
        self.declare_parameter('llm_extract_service', 'llm/extract_info')
        self.declare_parameter('llm_check_intent_service', 'llm/check_intent')
        self.declare_parameter('llm_clear_service', 'llm/clear_history')
        self.declare_parameter('llm_timeout_sec', 5.0)
        self.declare_parameter('processing_workers', 2)
        self.declare_parameter('max_queue_size', 50)

        # Read parameters
        self.asr_topic = self.get_parameter('asr_topic').get_parameter_value().string_value
        self.editor_topic = self.get_parameter('editor_topic').get_parameter_value().string_value
        self.llm_chat_service_name = self.get_parameter('llm_chat_service').get_parameter_value().string_value
        self.llm_generate_service_name = self.get_parameter('llm_generate_service').get_parameter_value().string_value
        self.llm_extract_service_name = self.get_parameter('llm_extract_service').get_parameter_value().string_value
        self.llm_check_intent_service_name = self.get_parameter('llm_check_intent_service').get_parameter_value().string_value
        self.llm_clear_service_name = self.get_parameter('llm_clear_service').get_parameter_value().string_value
        self.llm_timeout = float(self.get_parameter('llm_timeout_sec').get_parameter_value().double_value)
        self.processing_workers = self.get_parameter('processing_workers').get_parameter_value().integer_value
        self.max_queue_size = self.get_parameter('max_queue_size').get_parameter_value().integer_value

        self.get_logger().info(f"HubNode started. ASR topic: {self.asr_topic}, Editor topic: {self.editor_topic}")

        # Internal queue for ASR texts
        self._asr_queue = queue.Queue(maxsize=self.max_queue_size)

        # Subscribers & publishers
        self._asr_sub = self.create_subscription(String, self.asr_topic, self._asr_callback, 10)
        # Editor (schedule manager) expects std_msgs/String messages whose .data is JSON as in your examples
        self._editor_pub = self.create_publisher(String, self.editor_topic, 10)

        # Thread pool for processing
        self._executor_pool = ThreadPoolExecutor(max_workers=max(1, self.processing_workers))

        # LLM service clients
        self._llm_chat_client = self.create_client(StringForString, self.llm_chat_service_name)
        self._llm_generate_client = self.create_client(StringForString, self.llm_generate_service_name)
        self._llm_extract_client = self.create_client(StringForString, self.llm_extract_service_name)
        self._llm_check_client = self.create_client(StringForString, self.llm_check_intent_service_name)
        self._llm_clear_client = self.create_client(SetString, self.llm_clear_service_name)

        # Non-fatal short wait for availability
        short_wait = 5.0
        try:
            self._llm_generate_client.wait_for_service(timeout_sec=short_wait)
        except Exception:
            pass  # we'll handle availability at call time

        self._closed = False
        self._queue_watcher_thread = threading.Thread(target=self._queue_watcher, daemon=True)
        self._queue_watcher_thread.start()
        
        self._state = self.State.IDLE
        self._operation_cache = None
        self._text_cache = ''

        self.get_logger().info("HubNode initialized.")

    # -------------------------------
    # ASR 收集（来自你提供的 ASR 节点）
    # -------------------------------
    def _asr_callback(self, msg: String):
        """Callback to collect ASR text and enqueue for processing."""
        # code shows that msg.data is bare text
        try:
            text = msg.data.strip() if msg.data is not None else ''
            text = self._text_cache + text
        except Exception as e:
            self.get_logger().warn(f"Error reading ASR message: {e}")
            return

        if not text:
            self.get_logger().debug("Empty ASR text; ignoring.")
            return

        try:
            self._asr_queue.put_nowait({'text': text, 'received_ts': time.time()})
            self.get_logger().info(f"Enqueued ASR text: '{text}'")
        except queue.Full:
            self.get_logger().warn("ASR queue full; dropping transcription.")

    # -------------------------------
    # Queue watcher / worker
    # -------------------------------
    def _queue_watcher(self):
        self.get_logger().debug("Queue watcher thread started.")
        while rclpy.ok() and not self._closed:
            try:
                item = self._asr_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._executor_pool.submit(self._process_asr_item, item)
            try:
                self._asr_queue.task_done()
            except Exception:
                pass
        self.get_logger().debug("Queue watcher thread exiting.")

    def _process_asr_item(self, item: Dict[str, Any]):
        """
        Central processing pipeline for a single ASR transcription:
          1. (TODO) preprocessing
          2. call llm/generate_operation to obtain structured operation
          3. if failed to obtain structured op, try check_intent/extract_info fallback
          4. publish JSON request to schedule_manager via /schedule_manager/request
        """
        text = item.get('text', '')
        ts = item.get('received_ts', None)
        self.get_logger().info(f"Processing ASR: '{text}' received at {ts}")

        match self._state:
            case self.State.IDLE:
                result_str = self.extract_info(f'text: {text}, pre_confirmed: {self._operation_cache is not None }')
                result = json.loads(result_str)
                candidates = result['candidates']
                if len(candidates) == 0:
                    # ignore
                    pass
                else:
                    # might support multiple choices later
                    candidate = candidates[0]
                    if candidate['need_confirm']:
                        # TODO: vocal output?
                        self._state = self.State.NEED_CONFIRM
                        self._operation_cache = candidate
                    else:
                        self.dispatch_operation(candidate)
            case self.State.NEED_CONFIRM:
                result_str = self.check_intent(text)
                result = json.loads(result_str)
                match result['intent']:
                    case 'confirm':
                        self.dispatch_operation(self._operation_cache)
                    case 'deny':
                        self.clear_history()
                        self.State = self.State.IDLE
                    case 'edit':
                        self._state = self.State.IDLE
                        self._text_cache += text
                    case 'irrelevant':
                        # 相信大模型一定能把无关信息踢掉吧
                        self._text_cache += text
                        pass

    def dispatch_operation(self, candidate: Dict[str, Any]):
        '''
        This method clears history/cache and restores state to IDLE
        '''
        if candidate['intent'] == 'add':
            self.add_schedule(candidate)
        else:
            all_schedules = self.list_schedules()
            candidate.pop('need_confirm')
            candidate['all_schedules'] = all_schedules
            generated_operations = self.generate_operation(candidate)
            for operation in generated_operations:
                match operation['operation']:
                    case 'edit':
                        self.patch_schedule(operation)
                    case 'delete':
                        self.delete_schedule(operation)
                    case 'query':
                        self.query_schedule(operation)
        # TODO: vocal output?
        self.clear_history()
        self._state = self.State.IDLE

    def clear_history(self):
        self._operation_cache = None
        self.clear_history()
        self._state = self.State.IDLE
    # -------------------------------
    # Editor publishing helper
    # -------------------------------
    def _send_to_editor(self, operation: Dict[str, Any]):
        """
        Publish operation (dict) to the editor topic as JSON string in std_msgs/String.data.

        Expected final format examples (based on your samples):
        {"request_id":"1","op":"now","args":{}}
        {"request_id":"3","op":"add","args":{"schedule":{ ... }}}
        """
        if not isinstance(operation, dict):
            raise ValueError("operation must be a dict")

        request_id = make_request_id()
        operation['request_id'] = request_id
        payload = json.dumps(operation, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self._editor_pub.publish(msg)
        
    def request_timestamp(self):
        return self._send_to_editor({'op': 'now', 'args': {}})
    
    def list_schedules(self):
        return self._send_to_editor({'op': 'list', 'args': {}})
    
    def add_schedule(self, schedule: Dict[str, Any]):
        del schedule['intent']
        request_dict = {}
        request_dict['op'] = 'add'
        request_dict['args'] = {'schedule': schedule['new_schedule']}
        # TODO: gen id?
        return self._send_to_editor(request_dict)

    def delete_schedule(self, schedule: Dict[str, Any]):
        del schedule['intent']
        request_dict = {}
        request_dict['op'] = 'delete'
        request_dict['args'] = {'id': schedule['old_schedule']['id']}
        return self._send_to_editor(request_dict)
    
    def patch_schedule(self, schedule: Dict[str, Any]):
        del schedule['intent']
        request_dict = {}
        request_dict['op'] = 'update'
        request_dict['args'] = {'id': schedule['old_schedule']['id'], 'patch': schedule['new_schedule']}
        return self._send_to_editor(request_dict)
    
    def query_schedule(self, schedule: Dict[str, Any]):
        del schedule['intent']
        request_dict = {}
        request_dict['op'] = 'get'
        request_dict['args'] = {'id': schedule['old_schedule']['id']}
        return self._send_to_editor(request_dict)
    # -------------------------------
    # LLM service call helpers (StringForString and SetString)
    # -------------------------------
    def _call_string_for_string(self, client, text: str, timeout: float) -> Optional[str]:
        if client is None:
            self.get_logger().error("LLM client is None.")
            return None
        try:
            if not client.wait_for_service(timeout_sec=min(1.0, timeout)):
                self.get_logger().warn(f"Service {getattr(client, 'srv_name', '(unknown)')} not available.")
                return None
        except Exception:
            self.get_logger().warn("Exception during wait_for_service.")
            return None

        req = StringForString.Request()
        req.data = text
        self.get_logger().info(f"Calling LLM service {getattr(client, 'srv_name', '(unknown)')}: {text}")
        future = client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        except Exception as e:
            self.get_logger().error(f"Exception waiting for LLM future: {e}")
            return None
        if future.done():
            resp = future.result()
            return getattr(resp, 'data', None)
        else:
            self.get_logger().warn("LLM call timed out.")
            return None

    def send_to_llm(self, text: str, timeout: Optional[float] = None) -> Optional[str]:
        timeout = timeout if timeout is not None else self.llm_timeout
        return self._call_string_for_string(self._llm_chat_client, text, timeout)

    def generate_operation(self, text: str, timeout: Optional[float] = None) -> Optional[str]:
        timeout = timeout if timeout is not None else self.llm_timeout
        return self._call_string_for_string(self._llm_generate_client, text, timeout)

    def extract_info(self, text: str, timeout: Optional[float] = None) -> Optional[str]:
        timeout = timeout if timeout is not None else self.llm_timeout
        return self._call_string_for_string(self._llm_extract_client, text, timeout)

    def check_intent(self, text: str, timeout: Optional[float] = None) -> Optional[str]:
        timeout = timeout if timeout is not None else self.llm_timeout
        return self._call_string_for_string(self._llm_check_client, text, timeout)

    def clear_history(self, timeout: Optional[float] = None) -> bool:
        '''
        This method clears llm context, _operation_cache and restores state to IDLE
        '''
        self._operation_cache = None
        self._state = self.State.IDLE
        
        client = self._llm_clear_client
        if client is None:
            return False
        try:
            if not client.wait_for_service(timeout_sec=min(1.0, timeout or self.llm_timeout)):
                return False
        except Exception:
            return False
        req = SetString.Request()
        req.data = 'clear'
        future = client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=(timeout or self.llm_timeout))
        except Exception:
            return False
        if future.done():
            resp = future.result()
            return getattr(resp, 'success', False)
        return False
    # -------------------------------
    # Shutdown / cleanup
    # -------------------------------
    def destroy_node(self):
        self.get_logger().info("HubNode shutting down...")
        self._closed = True
        try:
            self._executor_pool.shutdown(wait=True)
        except Exception:
            pass
        if self._queue_watcher_thread.is_alive():
            self._queue_watcher_thread.join(timeout=1.0)
        super().destroy_node()
        self.get_logger().info("HubNode destroyed.")


def main(args=None):
    rclpy.init(args=args)
    node = HubNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
