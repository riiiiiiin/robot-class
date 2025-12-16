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
        self.declare_parameter('tts_service', 'tts_service') 
        
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
        self.tts_service_name = self.get_parameter('tts_service').get_parameter_value().string_value
        
        self.llm_timeout = float(self.get_parameter('llm_timeout_sec').get_parameter_value().double_value)
        self.processing_workers = self.get_parameter('processing_workers').get_parameter_value().integer_value
        self.max_queue_size = self.get_parameter('max_queue_size').get_parameter_value().integer_value

        self.get_logger().info(f"HubNode started. ASR topic: {self.asr_topic}, Editor topic: {self.editor_topic}")

        # Internal queue for ASR texts
        self._asr_queue = queue.Queue(maxsize=self.max_queue_size)

        # Subscribers & publishers
        self._asr_sub = self.create_subscription(String, self.asr_topic, self._asr_callback, 10)
        self._editor_pub = self.create_publisher(String, self.editor_topic, 10)

        # Thread pool for processing
        self._executor_pool = ThreadPoolExecutor(max_workers=max(1, self.processing_workers))

        # LLM service clients
        self._llm_chat_client = self.create_client(StringForString, self.llm_chat_service_name)
        self._llm_generate_client = self.create_client(StringForString, self.llm_generate_service_name)
        self._llm_extract_client = self.create_client(StringForString, self.llm_extract_service_name)
        self._llm_check_client = self.create_client(StringForString, self.llm_check_intent_service_name)
        self._llm_clear_client = self.create_client(SetString, self.llm_clear_service_name)
        
        # TTS service client
        self._tts_client = self.create_client(SetString, self.tts_service_name)

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

        self.get_logger().info("HubNode initialized.")

    # ---------------------------
    # TTS helper
    # ---------------------------
    def speak(self, text: str):
        """Call TTS service to speak text."""
        if not text:
            return
        
        # 简单检查服务是否就绪，不阻塞
        if self._tts_client.service_is_ready():
            req = SetString.Request()
            req.data = text
            self._tts_client.call_async(req)
            self.get_logger().info(f"Speaking: {text}")
        else:
            self.get_logger().warn("TTS service not ready, cannot speak.")
    # -------------------------------
    # ASR 收集（来自你提供的 ASR 节点）
    # -------------------------------
    def _asr_callback(self, msg: String):
        try:
            text = msg.data.strip() if msg.data is not None else ''
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
        text = item.get('text', '')
        ts = item.get('received_ts', None)
        self.get_logger().info(f"Processing ASR: '{text}' received at {ts}")

        match self._state:
            case self.State.IDLE:
                result_str = self.extract_info(f'text: {text}, pre_confirmed: {self._operation_cache is not None }')
                result = json.loads(result_str)
                candidates = result['candidates']
                if len(candidates) == 0:
                    # no candidate, just leave it in context
                    pass
                else:
                    candidate = candidates[0]
                    if candidate['need_confirm']:
                        self._state = self.State.NEED_CONFIRM
                        self._operation_cache = candidate
                        
                        intent = candidate.get('intent', 'unknown')
                        tts_text = "请确认一下。"
                        if intent == 'add':
                            new_s = candidate.get('new_schedule', {})
                            tts_text = f"您是想添加在{new_s.get('start_time', '')}，{new_s.get('description', '')}的日程吗？"
                        elif intent == 'delete':
                            old_s = candidate.get('old_schedule', {})
                            tts_text = f"您是想删除在{old_s.get('start_time', '')}，{old_s.get('description', '')}的日程吗？"
                        elif intent == 'edit':
                            old_s = candidate.get('old_schedule', {})
                            new_s = candidate.get('new_schedule', {})
                            tts_text = f"您是想将{old_s.get('start_time', '')}，{old_s.get('description', '')}的日程修改为{new_s.get('start_time', '')}，{new_s.get('description', '')}吗？"
                        elif intent == 'query':
                            old_s = candidate.get('old_schedule', {})
                            tts_text = f"您是想查询在{old_s.get('start_time', '')}，{old_s.get('description', '')}的日程吗？"
                        self.speak(tts_text)
                        
                    else:
                        self.dispatch_operation(candidate)
                        
            case self.State.NEED_CONFIRM:
                result_str = self.check_intent(text)
                result = json.loads(result_str)
                match result['intent']:
                    case 'confirm':
                        self.dispatch_operation(self._operation_cache)
                    case 'deny':
                        self.speak("好的，已取消。")
                        self.clear_history()
                    case 'edit':
                        # the last round contains the corrected information
                        # so the initial and corrected are both in context
                        # just tell llm to extract information from context
                        corrected_result_str = self.extract_info(f'text: "用户在之前的对话中，提出了进行日程操作，以及对细节的更正，请你综合上下文提取。", pre_confirmed: {True}')
                        corrected_result = json.loads(corrected_result_str)
                        corrected_candidates = corrected_result['candidates']
                        if len(corrected_candidates) == 0:
                            # no candidate, just leave it in context
                            pass
                        else:
                            corrected_candidate = corrected_candidates[0]
                            self.dispatch_operation(corrected_candidate)
                    case 'irrelevant':
                        # irrelevant, just leave it in context
                        pass

    def dispatch_operation(self, candidate: Dict[str, Any]):
        feedback_text = "操作已提交。"
        
        if candidate['intent'] == 'add':
            self.add_schedule(candidate)
            desc = candidate.get('new_schedule', {}).get('description', '日程')
            feedback_text = f"已添加日程：{desc}。"
        else:
            all_schedules = self.list_schedules()
            candidate.pop('need_confirm')
            candidate['all_schedules'] = all_schedules
            generated_operations_str = self.generate_operation(json.dumps(candidate))
            
            try:
                generated_operations = json.loads(generated_operations_str)
                generated_operations = generated_operations.get('candidates', []) if isinstance(generated_operations, dict) else generated_operations

                op_count = 0
                for operation in generated_operations:
                    op_type = operation.get('operation')
                    match op_type:
                        case 'edit':
                            self.patch_schedule(operation)
                            op_count += 1
                        case 'delete':
                            self.delete_schedule(operation)
                            op_count += 1
                        case 'query':
                            # query_schedule actually does nothing as we have provided all_schedules
                            self.query_schedule(operation)
                            feedback_text += f"您在{operation.get('start_time', '')}有{operation.get('description', '')}的日程。"

                if candidate['intent'] == 'delete':
                    feedback_text = f"已删除 {op_count} 个日程。"
                elif candidate['intent'] == 'edit':
                    feedback_text = "日程修改成功。"
            
            except Exception as e:
                self.get_logger().error(f"Error parsing/executing operations: {e}")
                feedback_text = "处理您的请求时遇到了一些问题。"

        self.speak(feedback_text)
        self.clear_history()
    # -------------------------------
    # Editor publishing helper
    # -------------------------------
    def _send_to_editor(self, operation: Dict[str, Any]):
        if not isinstance(operation, dict):
            raise ValueError("operation must be a dict")

        request_id = make_request_id()
        operation['request_id'] = request_id
        payload = json.dumps(operation, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self._editor_pub.publish(msg)
        return request_id # Return ID if needed
        
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

    def delete_schedule(self, operation: Dict[str, Any]):
        request_dict = {}
        request_dict['op'] = 'delete'
        request_dict['args'] = {'id': operation.get('id')}
        return self._send_to_editor(request_dict)
    
    def patch_schedule(self, operation: Dict[str, Any]):
        patch_data = {}
        for key in ['start_time', 'end_time', 'location', 'description']:
            if operation.get(key):
                patch_data[key] = operation[key]
                
        request_dict = {}
        request_dict['op'] = 'update'
        request_dict['args'] = {'id': operation.get('id'), 'patch': patch_data}
        return self._send_to_editor(request_dict)
    
    def query_schedule(self, operation: Dict[str, Any]):
        request_dict = {}
        request_dict['op'] = 'get'
        request_dict['args'] = {'id': operation.get('id')}
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