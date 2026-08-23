import React, { useState } from 'react';
import { Send, Play, RotateCcw, Box, Layers } from 'lucide-react';

interface Props {
  onSendPrompt: (prompt: string) => void;
  lastPrompt: string;
}

export const ControlPanel: React.FC<Props> = ({ onSendPrompt, lastPrompt }) => {
  const [inputPrompt, setInputPrompt] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputPrompt.trim()) return;
    onSendPrompt(inputPrompt.trim());
    setInputPrompt('');
  };

  const quickActions = [
    { label: 'Arrange All Blocks', prompt: 'arrange all boxes', icon: Layers, color: 'apple-button-primary' },
    { label: 'Stack Red Box', prompt: 'stack the red box', icon: Box, color: '' },
    { label: 'Stack Green Box', prompt: 'stack the green box', icon: Box, color: '' },
    { label: 'Stack Blue Box', prompt: 'stack the blue box', icon: Box, color: '' },
    { label: 'Stack Yellow Box', prompt: 'stack the yellow box', icon: Box, color: '' },
    { label: 'Reset Stack', prompt: 'reset stack', icon: RotateCcw, color: 'apple-button-danger' }
  ];

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Play size={20} color="var(--accent-orange)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>Natural Language Robot Dispatcher</h2>
        </div>
        <span className="badge badge-orange">ROS2 /user_prompt</span>
      </div>

      {/* Quick Action Preset Buttons */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '10px' }}>
        {quickActions.map((act, idx) => {
          const Icon = act.icon;
          return (
            <button
              key={idx}
              className={`apple-button ${act.color}`}
              onClick={() => onSendPrompt(act.prompt)}
              style={{ justifyContent: 'center', fontSize: '12px', padding: '10px' }}
            >
              <Icon size={14} />
              {act.label}
            </button>
          );
        })}
      </div>

      {/* Custom Prompt Text Box */}
      <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '10px', marginTop: '4px' }}>
        <input
          type="text"
          value={inputPrompt}
          onChange={(e) => setInputPrompt(e.target.value)}
          placeholder="e.g. 'stack the red box on the blue box'..."
          style={{
            flex: 1,
            padding: '12px 16px',
            borderRadius: '12px',
            background: 'rgba(0, 0, 0, 0.3)',
            border: '1px solid rgba(255, 255, 255, 0.15)',
            color: '#fff',
            fontSize: '13px',
            outline: 'none',
            fontFamily: 'inherit'
          }}
        />
        <button type="submit" className="apple-button apple-button-primary">
          <Send size={15} /> Dispatch
        </button>
      </form>

      {lastPrompt && (
        <div style={{ fontSize: '11px', color: 'var(--text-tertiary)', display: 'flex', alignItems: 'center', gap: '6px' }}>
          <span>Last Executed:</span>
          <code style={{ background: 'rgba(255,255,255,0.08)', padding: '2px 6px', borderRadius: '4px', color: 'var(--accent-teal)' }}>
            "{lastPrompt}"
          </code>
        </div>
      )}
    </div>
  );
};
