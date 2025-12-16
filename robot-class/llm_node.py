#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_define.srv import StringForString
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
            
        self.srv = self.create_service(StringForString, 'llm/chat', self.handle_llm_request)
        self.srv_extract_info = self.create_service(StringForString, 'llm/extract_info', self.handle_extract_info)
        self.srv_check_intent = self.create_service(StringForString, 'llm/check_intent', self.handle_check_intent)
        self.srv_generate_operation = self.create_service(StringForString, 'llm/generate_operation', self.handle_generate_operation)
        self.get_logger().info('llm_node ready, service: llm/chat, llm/extract_info, llm/check_intent, llm/generate_operation')
        self.client = OpenAI(
            api_key=os.environ.get('DEEPSEEK_API_KEY'),
            base_url="https://api.deepseek.com")

    def handle_llm_request(self, request, response, system_prompt = "You are a helpful assistant."):
        self.get_logger().info(f'Sending LLM request: "{request.data}"')
        
        result = self.client.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {"role": "system", "content": f"现在的时间是：{datetime.now()}。下面是你的任务描述：{system_prompt}"},
                {"role": "user", "content": request.data},
            ],
            stream=False
        )
        
        self.get_logger().info(f'Received LLM response: "{result.choices[0].message}"')

        response.data = result.choices[0].message.content

        return response

    def handle_extract_info(self, request, response):
        return self.handle_llm_request(request, response, self.prompt_extract_info)
    def handle_check_intent(self, request, response):
        return self.handle_llm_request(request, response, self.prompt_check_intent)
    def handle_generate_operation(self, request, response):
        return self.handle_llm_request(request, response, self.prompt_generate_operation)
    
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
