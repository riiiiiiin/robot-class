import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Dict, Any
import json
import uuid
from enum import Enum
import traceback

from service_define.srv import StringForString
from service_define.srv import SetString 

def make_request_id() -> str:
    """Generate a compact request id: <timestamp_ms>-<shortuuid>"""
    ts_ms = int(time.time() * 1000)
    short = uuid.uuid4().hex[:8]
    return f"{ts_ms}-{short}"

class HubNode(Node):
    class State(Enum):
        IDLE = 0
        NEED_CONFIRM = 1
    def __init__(self, node_name: str = 'hub_node'):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter('asr_topic', '/asr_result')
        self.declare_parameter('editor_topic', '/schedule_manager/request')
        self.declare_parameter('editor_topic_response', '/schedule_manager/response')
        self.declare_parameter('llm_chat_service', 'llm/chat')
        self.declare_parameter('llm_generate_service', 'llm/generate_operation')
        self.declare_parameter('llm_extract_service', 'llm/extract_info')
        self.declare_parameter('llm_check_intent_service', 'llm/check_intent')
        self.declare_parameter('llm_clear_service', 'llm/clear_history')
        self.declare_parameter('tts_service', 'tts_service') 
        
        self.declare_parameter('llm_timeout_sec', 10.0)
        self.declare_parameter('processing_workers', 2)

        # Read parameters
        self.asr_topic = self.get_parameter('asr_topic').get_parameter_value().string_value
        self.editor_topic = self.get_parameter('editor_topic').get_parameter_value().string_value
        self.editor_topic_response = self.get_parameter('editor_topic_response').get_parameter_value().string_value
        self.llm_chat_service_name = self.get_parameter('llm_chat_service').get_parameter_value().string_value
        self.llm_generate_service_name = self.get_parameter('llm_generate_service').get_parameter_value().string_value
        self.llm_extract_service_name = self.get_parameter('llm_extract_service').get_parameter_value().string_value
        self.llm_check_intent_service_name = self.get_parameter('llm_check_intent_service').get_parameter_value().string_value
        self.llm_clear_service_name = self.get_parameter('llm_clear_service').get_parameter_value().string_value
        self.tts_service_name = self.get_parameter('tts_service').get_parameter_value().string_value
        
        self.llm_timeout = float(self.get_parameter('llm_timeout_sec').get_parameter_value().double_value)
        self.processing_workers = self.get_parameter('processing_workers').get_parameter_value().integer_value

        self.get_logger().info(f"HubNode started. ASR topic: {self.asr_topic}, Editor topic: {self.editor_topic}")

        # Subscribers & publishers
        self._asr_sub = self.create_subscription(String, self.asr_topic, self._asr_callback, 10)
        self._editor_pub = self.create_publisher(String, self.editor_topic, 10)
        self._editor_response_sub = self.create_subscription(String, self.editor_topic_response, self._editor_response_callback, 10)

        # Thread pool for processing
        self._executor_pool = ThreadPoolExecutor(max_workers=max(1, self.processing_workers))
        
        # Lock for thread sync (blocking) wait
        self._pending_lock = threading.Lock()
        self._pending_events: Dict[str, Any] = {}
        self._pending_responses: Dict[str, Dict[str, Any]] = {}

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
            self.get_logger().info(f"_asr_callback received: {msg.data!r}")
            text = msg.data.strip() if msg.data is not None else ''
        except Exception as e:
            traceback.print_exc()
            self.get_logger().warn(f"Error reading ASR message: {e}")
            return

        if not text:
            self.get_logger().debug("Empty ASR text; ignoring.")
            return

        try:
            self._process_asr_item({'text': text, 'received_ts': time.time()})
        except Exception as e:
            traceback.print_exc()
            self.get_logger().warn(f"Error processing ASR text: {e}")


    def _process_asr_item(self, item: Dict[str, Any]):
        """
        非阻塞异步版：发起 extract_info_async，然后在回调中继续处理，
        所有后续调用都使用 *_async 的接口。
        """
        try:
            text = item.get('text', '')
            ts = item.get('received_ts', None)
            self.get_logger().info(f"Processing ASR (async): '{text}' received at {ts}")

            def _on_extract_done(result_str):
                try:
                    if not result_str:
                        result = {'candidates': []}
                    else:
                        try:
                            result = json.loads(result_str)
                        except Exception as e:
                            self.get_logger().warn(f"Error parsing extract_info result: {e}")
                            result = {'candidates': []}

                    
                    candidates = result.get('candidates', [])
                    if len(candidates) == 0:
                        return
                    else:
                        candidate = candidates[0]
                        if candidate.get('need_confirm'):
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
                            # no confirm required -> dispatch asynchronously
                            try:
                                # dispatch_operation_async will do list_schedules_async & generate op async
                                self.dispatch_operation_async(candidate)
                            except Exception as e:
                                self.get_logger().error(f"dispatch_operation_async error: {e}")
                except Exception:
                    traceback.print_exc()
            
            def _on_check_done(check_str):
                try:
                    if not check_str:
                        return
                    res = json.loads(check_str)
                    intent = res.get('intent')
                    print(intent)
                    if intent == 'confirm':
                        self.dispatch_operation_async(self._operation_cache)
                    elif intent == 'deny':
                        self.speak("好的，已取消。")
                        # clear_history_async no blocking
                        self.clear_history_async()
                    elif intent == 'edit':
                        # ask LLM to extract corrected info (async)
                        def _on_corrected_extract(corrected_str):
                            try:
                                if not corrected_str:
                                    return
                                corrected_json = json.loads(corrected_str)
                                corrected_candidates = corrected_json.get('candidates', [])
                                if len(corrected_candidates) == 0:
                                    return
                                corrected_candidate = corrected_candidates[0]
                                self.dispatch_operation_async(corrected_candidate)
                            except Exception as e:
                                self.get_logger().error(f"corrected_extract parse error: {e}")

                        self.extract_info_async(
                            'text: "用户在之前的对话中，提出了进行日程操作，以及对细节的更正，请你综合上下文提取。", pre_confirmed: True',
                            _on_corrected_extract
                        )
                    else:
                        # irrelevant -> ignore
                        pass
                except Exception:
                    traceback.print_exc()               
            if self._state == self.State.IDLE:
                self.extract_info_async(f'text: {text}, pre_confirmed: {self._operation_cache is not None }', _on_extract_done)
            elif self._state == self.State.NEED_CONFIRM:
                self.check_intent_async(text, _on_check_done)

        except Exception:
            traceback.print_exc()


    def dispatch_operation_async(self, candidate: Dict[str, Any]):
        """
        Non-blocking dispatch of an operation represented by candidate.
        For 'add' it publishes add request to editor.
        For others, it will first fetch schedules async then ask generate_operation_async
        and then apply generated operations (publish to editor). All are non-blocking.
        """
        try:
            feedback_text = "操作已提交。"

            if candidate.get('intent') == 'add':
                # publish add without waiting
                self.add_schedule(candidate.get('new_schedule', {}))
                desc = candidate.get('new_schedule', {}).get('description', '日程')
                feedback_text = f"已添加日程：{desc}。"
                self.speak(feedback_text)
                self.clear_history_async()
                return

            # for delete/edit/query: need schedules first
            def _on_list(schedules):
                try:
                    # attach schedules to candidate and generate operations
                    candidate2 = dict(candidate)
                    candidate2.pop('need_confirm', None)
                    candidate2['all_schedules'] = schedules or []

                    def _on_generate(gen_str):
                        try:
                            if not gen_str:
                                self.get_logger().warn("generate_operation returned None")
                                self.speak("处理请求时出错。")
                                self.clear_history_async()
                                return
                            try:
                                generated_operations = json.loads(gen_str)
                            except Exception:
                                generated_operations = gen_str

                            # normalize
                            if isinstance(generated_operations, dict):
                                generated_operations = generated_operations.get('candidates', []) or []

                            op_count = 0
                            feedback = []

                            for operation in generated_operations:
                                op_type = operation.get('operation')
                                if op_type == 'edit':
                                    self.patch_schedule(operation)
                                    op_count += 1
                                elif op_type == 'delete':
                                    self.delete_schedule(operation)
                                    op_count += 1
                                elif op_type == 'query':
                                    # publish a get request but don't wait
                                    def _on_get(x):
                                        pass
                                    self.query_schedule_async(operation, _on_get)
                                    feedback.append(f"您在{operation.get('start_time','')}有{operation.get('description','')}的日程。")
                                    

                            if candidate2.get('intent') == 'delete':
                                feedback_text_local = f"已删除 {op_count} 个日程。"
                            elif candidate2.get('intent') == 'edit':
                                feedback_text_local = "日程修改成功。"
                            else:
                                feedback_text_local = "操作已完成。"

                            # speak feedback
                            self.speak("。".join([feedback_text_local] + feedback))
                        except Exception:
                            traceback.print_exc()
                            self.speak("处理您的请求时遇到了一些问题。")
                        finally:
                            self.clear_history_async()

                    # generate operations (async)
                    self.generate_operation_async(json.dumps(candidate2), _on_generate)

                except Exception:
                    traceback.print_exc()
                    self.speak("处理请求时发生异常。")
                    self.clear_history_async()

            # start by retrieving schedules asynchronously
            self.list_schedules_async(_on_list)

        except Exception:
            traceback.print_exc()
            self.speak("分发操作时出错。")
            self.clear_history_async()

    # -------------------------------
    # Editor publishing helper
    # -------------------------------
    def _send_to_editor(self, operation: Dict[str, Any], response_cb: Optional[Any] = None) -> str:
        """
        Publish an editor request. If response_cb is provided, it will be called as:
            response_cb(response_payload_dict)
        when a corresponding response with same request_id is received.
        This is non-blocking.
        Returns request_id.
        """
        if not isinstance(operation, dict):
            raise ValueError("operation must be a dict")

        request_id = make_request_id()
        operation['request_id'] = request_id
        payload = json.dumps(operation, ensure_ascii=False)
        msg = String()
        msg.data = payload

        # If a callback is provided, register it so _editor_response_callback can call it.
        if response_cb is not None:
            with self._pending_lock:
                self._pending_events[str(request_id)] = response_cb

        self._editor_pub.publish(msg)
        self.get_logger().info(f"Published to editor op={operation.get('op')} request_id={request_id}")
        return request_id
    
    def _editor_response_callback(self, msg: String):
        """
        Called when editor publishes a response. Will route the response to
        a registered callback if present, otherwise store result in _pending_responses.
        """
        try:
            if not msg or not msg.data:
                return
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
            request_id = payload.get("request_id")
            if not request_id:
                return

            with self._pending_lock:
                # If a callback registered, pop and call it (call outside lock)
                cb = self._pending_events.pop(str(request_id), None)
                if cb:
                    # call the callback asynchronously (to avoid blocking subscriber thread)
                    try:
                        # schedule the callback on executor pool to avoid long work in subscriber thread
                        self._executor_pool.submit(cb, payload)
                    except Exception as e:
                        # fallback: call directly (best effort)
                        try:
                            cb(payload)
                        except Exception:
                            traceback.print_exc()
                    return
                # else, no callback registered: remember response for later retrieval
                self._pending_responses[str(request_id)] = payload

        except Exception as e:
            traceback.print_exc()
            self.get_logger().error(f"_on_editor_response parse error: {e}")
    
    def request_timestamp_async(self, done_cb: Any, timeout: float = 5.0):
        """
        Request current timestamp from editor and call done_cb(result_or_None).
        result is resp.get('data') on success, or None on error/timeout.
        """
        def _on_resp(payload):
            try:
                if not payload or not payload.get("ok"):
                    done_cb(None)
                    return
                done_cb(payload.get("data"))
            except Exception as e:
                self.get_logger().error(f"request_timestamp_async callback error: {e}")
                done_cb(None)

        rid = self._send_to_editor({'op': 'now', 'args': {}}, response_cb=_on_resp)
        # no blocking here; if you want to implement timeout behavior, you can schedule a timer that calls done_cb(None) after timeout if not called.

    def list_schedules_async(self, done_cb: Any, timeout: float = 5.0):
        """
        Ask editor for list of schedules, then call done_cb(schedules_list_or_None).
        """
        def _on_resp(payload):
            try:
                if not payload or not payload.get("ok"):
                    done_cb(None)
                    return
                done_cb(payload.get("data", {}).get("schedules"))
            except Exception as e:
                self.get_logger().error(f"list_schedules_async callback error: {e}")
                done_cb(None)

        rid = self._send_to_editor({'op': 'list', 'args': {}}, response_cb=_on_resp)

    def query_schedule_async(self, operation: Dict[str, Any], done_cb: Any, timeout: float = 5.0):
        """
        Query single schedule by id; calls done_cb(schedule_or_None)
        """
        def _on_resp(payload):
            try:
                if not payload or not payload.get("ok"):
                    done_cb(None)
                    return
                done_cb(payload.get("data", {}).get("schedule"))
            except Exception as e:
                self.get_logger().error(f"query_schedule_async callback error: {e}")
                done_cb(None)

        request_dict = {'op': 'get', 'args': {'id': operation.get('id')}}
        self._send_to_editor(request_dict, response_cb=_on_resp)

    
    def add_schedule(self, operation: Dict[str, Any]):
        new_data = {}
        for key in ['start_time', 'end_time', 'place', 'description']:
            if operation.get(key):
                new_data[key] = operation[key]
        
        request_dict = {}
        request_dict['op'] = 'add'
        request_dict['args'] = {'schedule': new_data}
        return self._send_to_editor(request_dict)

    def delete_schedule(self, operation: Dict[str, Any]):
        request_dict = {}
        request_dict['op'] = 'delete'
        request_dict['args'] = {'id': operation.get('id')}
        return self._send_to_editor(request_dict)
    
    def patch_schedule(self, operation: Dict[str, Any]):
        patch_data = {}
        for key in ['start_time', 'end_time', 'place', 'description']:
            if operation.get(key):
                patch_data[key] = operation[key]
                
        request_dict = {}
        request_dict['op'] = 'update'
        request_dict['args'] = {'id': operation.get('id'), 'patch': patch_data}
        return self._send_to_editor(request_dict)

    # -------------------------------
    # LLM service call helpers (StringForString and SetString)
    # -------------------------------
    def _call_string_for_string_async(self, client, text: str, done_cb: Any, timeout: Optional[float] = None):
        """
        Non-blocking LLM call. Calls done_cb(result_str_or_None).
        If client.service_is_ready() == False, calls done_cb(None) immediately.
        """
        timeout = timeout if timeout is not None else self.llm_timeout

        print(f'client: {client}, text: {text}')
        if client is None:
            self.get_logger().error("_call_string_for_string_async: client is None.")
            done_cb(None)
            return

        try:
            if not client.service_is_ready():
                self.get_logger().warn(f"LLM service {getattr(client, 'srv_name', '(unknown)')} not ready.")
                done_cb(None)
                return
        except Exception:
            # service_is_ready might throw on some setups; proceed to call anyway
            self.get_logger().warn("Exception checking service_is_ready; proceeding to call.")

        req = StringForString.Request()
        req.data = text
        try:
            future = client.call_async(req)
        except Exception as e:
            self.get_logger().error(f"LLM client.call_async failed: {e}")
            done_cb(None)
            return

        # register done callback
        def _on_done(fut):
            try:
                resp = fut.result()
                done_cb(getattr(resp, 'data', None))
            except Exception as e:
                self.get_logger().error(f"LLM async future error: {e}")
                done_cb(None)

        # Some rclpy futures support add_done_callback; this works for most versions
        try:
            future.add_done_callback(lambda fut: _on_done(fut))
        except Exception:
            traceback.print_exc()

    def extract_info_async(self, text: str, done_cb: Any):
        return self._call_string_for_string_async(self._llm_extract_client, text, done_cb, timeout=self.llm_timeout)

    def check_intent_async(self, text: str, done_cb: Any):
        return self._call_string_for_string_async(self._llm_check_client, text, done_cb, timeout=self.llm_timeout)

    def generate_operation_async(self, text: str, done_cb: Any):
        return self._call_string_for_string_async(self._llm_generate_client, text, done_cb, timeout=self.llm_timeout)

    def send_to_llm_async(self, text: str, done_cb: Any):
        return self._call_string_for_string_async(self._llm_chat_client, text, done_cb, timeout=self.llm_timeout)


    def clear_history_async(self, done_cb: Optional[Any] = None, timeout: Optional[float] = None):
        """
        Clear local state immediately. Optionally call LLM clear_history service,
        then call done_cb(success_bool) if provided.
        """
        self._operation_cache = None
        self._state = self.State.IDLE

        client = self._llm_clear_client
        if client is None:
            if done_cb:
                done_cb(False)
            return

        try:
            if not client.service_is_ready():
                self.get_logger().warn("clear_history_async: clear service not ready.")
                if done_cb:
                    done_cb(False)
                return
        except Exception:
            # ignore and attempt call
            pass

        req = SetString.Request()
        req.data = 'clear'
        try:
            future = client.call_async(req)
        except Exception as e:
            self.get_logger().error(f"clear_history_async call_async failed: {e}")
            if done_cb:
                done_cb(False)
            return

        def _on_done(fut):
            try:
                resp = fut.result()
                ok = getattr(resp, 'success', False)
                if done_cb:
                    done_cb(ok)
            except Exception as e:
                self.get_logger().error(f"clear_history_async future error: {e}")
                if done_cb:
                    done_cb(False)

        try:
            future.add_done_callback(lambda fut: _on_done(fut))
        except Exception:
            traceback.print_exc()

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
        super().destroy_node()
        self.get_logger().info("HubNode destroyed.")


def main(args=None):
    rclpy.init(args=args)
    node = HubNode()
    executor = MultiThreadedExecutor(num_threads=4)
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