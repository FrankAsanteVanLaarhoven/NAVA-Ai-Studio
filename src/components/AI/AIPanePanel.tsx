import React, { useState, useRef, useEffect } from 'react';
import './AIPanePanel.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

export const AIPanePanel: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      role: 'assistant',
      content: '👋 Hello! I\'m your NAVΛ AI Assistant. I can help you with:\n\n• Understanding Van Laarhoven Navigation Calculus\n• Writing and optimizing ⋋ expressions\n• Debugging navigation algorithms\n• Explaining mathematical concepts\n• Code suggestions and completions\n\nHow can I assist you today?',
      timestamp: new Date(),
    },
  ]);

  const [input, setInput] = useState('');
  const [isThinking, setIsThinking] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim() || isThinking) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsThinking(true);

    // Simulate AI response
    await new Promise(resolve => setTimeout(resolve, 1500));

    const aiResponse: Message = {
      id: (Date.now() + 1).toString(),
      role: 'assistant',
      content: generateAIResponse(input),
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, aiResponse]);
    setIsThinking(false);
  };

  const generateAIResponse = (query: string): string => {
    const lowerQuery = query.toLowerCase();

    if (lowerQuery.includes('⋋') || lowerQuery.includes('lambda')) {
      return '**Van Laarhoven Lambda (⋋)**\n\nThe ⋋ operator represents a navigation transformation in VNC. Here\'s how it works:\n\n```vnc\n⋋ φ: M → N\n```\n\nThis defines a navigation mapping from manifold M to manifold N. You can use it to:\n\n1. Define navigation fields\n2. Compose transformations\n3. Optimize paths through geometric spaces\n\nWould you like me to show you a specific example?';
    }

    if (lowerQuery.includes('optimize') || lowerQuery.includes('performance')) {
      return '**Performance Optimization Tips**\n\n1. **Use Memoization**: Cache computed navigation paths\n2. **Vectorize Operations**: Leverage SIMD for manifold computations\n3. **Parallel Processing**: Split large navigation fields across threads\n4. **GPU Acceleration**: Use WASM-SIMD for browser-based calculations\n\nHere\'s an optimized pattern:\n\n```vnc\n@memoize\n⋋ fast_nav = parallel_map(nav_field)\n```';
    }

    if (lowerQuery.includes('error') || lowerQuery.includes('debug')) {
      return '**Debugging Navigation Code**\n\n🔍 Common issues:\n\n1. **Type Mismatches**: Ensure manifold dimensions align\n2. **Singularities**: Check for division by zero in metric tensors\n3. **Convergence**: Verify optimization parameters\n\nTry adding:\n```vnc\n@debug_trace\n⋋ my_navigation_fn\n```\n\nThis will show step-by-step execution.';
    }

    return `I understand you're asking about: "${query}"\n\nLet me help you with that. The Van Laarhoven Navigation Calculus provides powerful abstractions for:\n\n• **Geometric computations** on manifolds\n• **Path optimization** through configuration spaces\n• **Energy minimization** in navigation fields\n\nCould you provide more specific details about what you'd like to accomplish?`;
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const insertSymbol = (symbol: string) => {
    setInput(prev => prev + symbol);
  };

  return (
    <div className="ai-pane-panel">
      <div className="ai-pane-header">
        <div className="ai-pane-title">
          <span className="ai-icon">🤖</span>
          <span>NAVΛ AI Assistant</span>
        </div>
        <div className="ai-status">
          <span className={`status-indicator ${isThinking ? 'thinking' : 'ready'}`}></span>
          <span className="status-text">{isThinking ? 'Thinking...' : 'Ready'}</span>
        </div>
      </div>

      <div className="ai-messages">
        {messages.map((message) => (
          <div key={message.id} className={`ai-message ${message.role}`}>
            <div className="message-avatar">
              {message.role === 'user' ? '👤' : '🤖'}
            </div>
            <div className="message-content">
              <div className="message-header">
                <span className="message-role">
                  {message.role === 'user' ? 'You' : 'AI Assistant'}
                </span>
                <span className="message-time">
                  {message.timestamp.toLocaleTimeString([], { 
                    hour: '2-digit', 
                    minute: '2-digit' 
                  })}
                </span>
              </div>
              <div className="message-text">
                {message.content.split('\n').map((line, i) => (
                  <React.Fragment key={i}>
                    {line}
                    {i < message.content.split('\n').length - 1 && <br />}
                  </React.Fragment>
                ))}
              </div>
            </div>
          </div>
        ))}
        {isThinking && (
          <div className="ai-message assistant thinking-indicator">
            <div className="message-avatar">🤖</div>
            <div className="thinking-dots">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="ai-input-area">
        <div className="symbol-toolbar">
          <button onClick={() => insertSymbol('⋋')} title="Van Laarhoven Lambda">⋋</button>
          <button onClick={() => insertSymbol('→')} title="Arrow">→</button>
          <button onClick={() => insertSymbol('∇')} title="Nabla">∇</button>
          <button onClick={() => insertSymbol('∫')} title="Integral">∫</button>
          <button onClick={() => insertSymbol('φ')} title="Phi">φ</button>
        </div>
        <div className="input-container">
          <textarea
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Ask me anything about NAVΛ or VNC..."
            className="ai-input"
            rows={3}
          />
          <button 
            onClick={handleSend} 
            disabled={!input.trim() || isThinking}
            className="send-btn"
            title="Send (Enter)"
          >
            {isThinking ? '⏳' : '📤'}
          </button>
        </div>
      </div>
    </div>
  );
};

