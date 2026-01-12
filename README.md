## Quick Start
1. Set your Deepseek Api Key in setup.bash
2. Start schedule manager:
   ```
   source setup.bash
   python schedule/schedule_manager.py
   ```
3. Start asr node:
   ```
   source setup.bash
   python asr_node.py
   ```
4. Start tts node:
   ```
   source setup.bash
   python tts_node.py
   ```
5. Start llm node:
   ```
   source setup.bash
   python llm_node.py
   ```
6. Start hub node:
   ```
   source setup.bash
   python hub_node.py
   ```
7. Start notification node:
   ```
   source setup.bash
   python notification_node.py
   ```
8. Start UI node, note that this node should be started on a shell with graphics
   ```
   source setup.bash
   python UI_node.py
   ```
