#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_define.srv import StringForString
from service_define.srv import SetString
import threading
import json
import os
import argparse
from datetime import datetime
from openai import OpenAI

def load_prompt(prompt_path):
    with open(prompt_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        return ''.join([line.strip() for line in lines])

class LLMNode(Node):
    def __init__(self, prompt_dir):
        super().__init__('llm_node')
        self.prompt_extract_info = load_prompt(prompt_dir + '/prompt_extract_info.txt')
        self.prompt_check_intent = load_prompt(prompt_dir + '/prompt_check_intent.txt')
        self.prompt_generate_operation = load_prompt(prompt_dir + '/prompt_generate_operation.txt')
            
        self.srv_chat = self.create_service(StringForString, 'llm/chat', self._handle_llm_request)
        self.srv_extract_info = self.create_service(StringForString, 'llm/extract_info', self._handle_extract_info)
        self.srv_check_intent = self.create_service(StringForString, 'llm/check_intent', self._handle_check_intent)
        self.srv_generate_operation = self.create_service(StringForString, 'llm/generate_operation', self._handle_generate_operation)
        self.srv_clear_history = self.create_service(SetString, 'llm/clear_history', self._handle_clear_history)
        self.get_logger().info('llm_node ready, service: llm/chat, llm/extract_info, llm/check_intent, llm/generate_operation, llm/clear_history')
        self.client = OpenAI(
            api_key=os.environ.get('DEEPSEEK_API_KEY'),
            base_url="https://api.deepseek.com")
        
        self.messages = []

    def _handle_clear_history(self, request, response):
        self.messages = []
        response.success = True
        return response
    
    def _append_message(self, message):
        if len(self.messages) > 10:
            self.messages = self.messages[2:]
            self.get_logger().info('Too many messages, poped 2 messages')
        self.messages.append(message)
    
    def _handle_llm_request(self, request, response, system_prompt = "You are a helpful assistant."):
        self.get_logger().info(f'Sending LLM request: "{request.data}"')
        
        self._append_message({"role": "user", "content": request.data})
        
        messages = [{"role": "system", "content": f"{system_prompt.replace("[CURRENT_TIME]", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))}"},]
        messages.extend(self.messages)
        
        result = self.client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            stream=False
        )
        
        self._append_message(result.choices[0].message)
        self.get_logger().info(f'Received LLM response: "{result.choices[0].message}"')
        response.data = result.choices[0].message.content

        return response

    def _handle_extract_info(self, request, response):
        return self._handle_llm_request(request, response, self.prompt_extract_info)
    def _handle_check_intent(self, request, response):
        return self._handle_llm_request(request, response, self.prompt_check_intent)
    def _handle_generate_operation(self, request, response):
        return self._handle_llm_request(request, response, self.prompt_generate_operation)
    
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prompt_dir', type=str, default='./llm')
    args = parser.parse_args()
    rclpy.init()
    node = LLMNode(args.prompt_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
