(function () {
  const TEXT_AGENT_START_GUIDE = [
    '网页只能连接 text-agent，不能自动启动虚拟机里的服务。',
    '',
    '请在运行仿真的 Ubuntu 虚拟机中打开新终端，依次执行：',
    '  cd /home/robot/ReBot_Arm_DigitalTwin_RS',
    "  export DASHSCOPE_API_KEY='替换成你的 Key'",
    "  export REBOTARM_LLM_MODEL='qwen-plus'",
    '  ./scripts/start_rs_text_agent.sh',
    '',
    '保持该终端运行。看到 listening on http://0.0.0.0:8082 后，可在另一终端检查：',
    '  curl http://127.0.0.1:8082/health',
    '',
    '返回包含 "ok": true 后，回到本页再次点击“连接 AI 助手”。',
    '仿真/MCP 也需要保持运行（通常先执行 ./scripts/start_rs_sim.sh）。'
  ].join('\n');

  class ReBotLLMUI {
    constructor() {
      this.started = false;
      this.config = { textAgentUrl: '', mcpUrl: '' };
      this.elements = {};
    }

    init() {
      this.elements = {
        status: document.getElementById('llm-status'),
        chatMessages: document.getElementById('llm-chat-messages'),
        input: document.getElementById('llm-input'),
        sendBtn: document.getElementById('llm-send'),
        startBtn: document.getElementById('llm-start'),
        stopBtn: document.getElementById('llm-stop'),
        message: document.getElementById('llm-message')
      };

      if (!this.elements.status) {
        console.error('LLM UI elements not found');
        return;
      }

      this.elements.startBtn.addEventListener('click', () => this.handleStart());
      this.elements.stopBtn.addEventListener('click', () => this.handleStop());
      this.elements.sendBtn.addEventListener('click', () => this.handleSend());
      this.elements.input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          this.handleSend();
        }
      });

      // 初始加载配置（仅用于显示）
      fetch('/api/mcp/config').then(r => r.json()).then(cfg => {
        this.config = cfg;
        this.elements.message.textContent = `连接地址: ${cfg.textAgentUrl}  ·  MCP: ${cfg.mcpUrl}。首次使用请展开下方启动说明。`;
      }).catch(() => {
        this.elements.message.textContent = '加载配置失败';
      });

      this.updateStatus('未启动');
    }

    updateStatus(status) {
      this.elements.status.textContent = status;
      let className = 'mini-pill ';
      if (status === '已启动') className += 'online';
      else if (status === '启动中...') className += 'warn';
      else if (status === '启动失败') className += 'error';
      else className += 'offline';
      this.elements.status.className = className;
    }

    async handleStart() {
      console.log('[LLM UI] handleStart');
      this.elements.startBtn.disabled = true;
      this.updateStatus('启动中...');
      this.elements.message.textContent = '正在连接 text-agent...';
      this.addMessage('system', '正在连接虚拟机中的 text-agent 服务...');

      try {
        // 健康检查
        const res = await fetch('/api/llm/health');
        const data = await res.json().catch(() => ({}));
        if (!res.ok || data.ok === false) {
          throw new Error(data.error || `text-agent 返回 HTTP ${res.status}`);
        }

        this.started = true;
        this.updateStatus('已启动');
        this.elements.message.textContent = 'text-agent 已连接，可以开始对话。';
        this.elements.stopBtn.disabled = false;
        this.elements.input.disabled = false;
        this.elements.sendBtn.disabled = false;
        this.addMessage('system', '已连接到 text-agent。\n直接输入中文指令即可，例如：\n• 查询机械臂状态\n• 移动到 X=0.3 Y=0 Z=0.3\n• 打开夹爪\n• 抓取红色物块');
        this.elements.input.focus();
      } catch (e) {
        console.error('[LLM UI] start failed:', e);
        this.updateStatus('启动失败');
        this.elements.message.textContent = `连接失败: ${e.message}`;
        this.addMessage('error', `连接失败: ${e.message}\n\n${TEXT_AGENT_START_GUIDE}`);
        this.elements.startBtn.disabled = false;
      }
    }

    handleStop() {
      this.started = false;
      this.updateStatus('未启动');
      this.elements.message.textContent = '已停止。';
      this.elements.stopBtn.disabled = true;
      this.elements.input.disabled = true;
      this.elements.sendBtn.disabled = true;
      this.addMessage('system', '已停止对话。');
      this.elements.startBtn.disabled = false;

      // 通知后端清空上下文（如果有 reset 端点）
      fetch('/api/llm/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: '__reset__', reset: true })
      }).catch(() => {});
    }

    async handleSend() {
      const text = this.elements.input.value.trim();
      if (!text) return;
      if (!this.started) {
        this.addMessage('error', '请先点击"启动 AI 助手"');
        return;
      }

      this.elements.input.value = '';
      this.addMessage('user', text);
      this.addMessage('loading', '思考中...');
      this.elements.sendBtn.disabled = true;

      try {
        const res = await fetch('/api/llm/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ text })
        });

        this.removeLoadingMessage();

        if (!res.ok) {
          const errText = await res.text();
          this.addMessage('error', `HTTP ${res.status}: ${errText.substring(0, 200)}`);
          return;
        }

        const data = await res.json();
        if (data.ok === false) {
          this.addMessage('error', `错误: ${data.error || '未知错误'}`);
          return;
        }

        // 显示回复
        if (data.text) {
          this.addMessage('assistant', data.text);
        } else {
          this.addMessage('assistant', '(无回复)');
        }

        // 显示工具调用过程
        const events = data.events || [];
        for (const evt of events) {
          if (evt.type === 'tool') {
            this.addMessage('tool', `🔧 ${evt.name}\n  参数: ${JSON.stringify(evt.arguments)}\n  结果: ${JSON.stringify(evt.result).substring(0, 200)}`);
          } else if (evt.type === 'info') {
            this.addMessage('info', evt.message);
          } else if (evt.type === 'error') {
            this.addMessage('error', evt.message);
          }
        }
      } catch (e) {
        this.removeLoadingMessage();
        this.addMessage('error', `请求失败: ${e.message}`);
      } finally {
        this.elements.sendBtn.disabled = false;
        this.elements.input.focus();
      }
    }

    addMessage(role, content) {
      const div = document.createElement('div');
      div.className = `llm-message llm-message-${role}`;
      if (content && content.includes('\n')) {
        div.style.whiteSpace = 'pre-wrap';
      }
      div.textContent = content;
      this.elements.chatMessages.appendChild(div);
      this.elements.chatMessages.scrollTop = this.elements.chatMessages.scrollHeight;
    }

    removeLoadingMessage() {
      const loading = this.elements.chatMessages.querySelector('.llm-message-loading');
      if (loading) loading.remove();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      window.llmUI = new ReBotLLMUI();
      window.llmUI.init();
    });
  } else {
    window.llmUI = new ReBotLLMUI();
    window.llmUI.init();
  }
})();
