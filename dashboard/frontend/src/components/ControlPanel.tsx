import React, { useState } from 'react';
import { Send, Play, RotateCcw, Layers, ArrowUp } from 'lucide-react';

interface Props {
  onSendPrompt: (prompt: string) => void;
  lastPrompt: string;
}

const COLORS = ['red', 'green', 'blue', 'yellow'] as const;

// Prompts verified against stacking_controller.py prompt_callback parsing
export const ControlPanel: React.FC<Props> = ({ onSendPrompt, lastPrompt }) => {
  const [inputPrompt, setInputPrompt] = useState('');
  const [pickColor, setPickColor] = useState<string>('green');
  const [placeColor, setPlaceColor] = useState<string>('yellow');

  const dispatch = (prompt: string) => {
    if (!prompt.trim()) return;
    onSendPrompt(prompt.trim());
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    dispatch(inputPrompt);
    setInputPrompt('');
  };

  // Fires on pointer-down: feedback at the moment of intent, not release
  const quickActions = [
    { label: 'Arrange All', prompt: 'arrange all blocks', icon: Layers, primary: true },
    { label: 'Reset Scene', prompt: 'reset', icon: RotateCcw, primary: false },
  ];

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Play size={20} color="var(--accent-orange)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>Robot Dispatcher</h2>
        </div>
        <span className="badge badge-orange">ROS2 /user_prompt</span>
      </div>

      {/* Quick actions */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '10px' }}>
        {quickActions.map((act) => {
          const Icon = act.icon;
          return (
            <button
              key={act.label}
              className={`apple-button ${act.primary ? 'apple-button-primary' : 'apple-button-danger'}`}
              onPointerDown={() => dispatch(act.prompt)}
              style={{ justifyContent: 'center', fontSize: '13px', padding: '11px' }}
            >
              <Icon size={14} />
              {act.label}
            </button>
          );
        })}
      </div>

      {/* Pick & place pair builder */}
      <div style={{
        display: 'flex', alignItems: 'center', gap: '8px',
        padding: '12px', borderRadius: '12px',
        background: 'rgba(0, 0, 0, 0.25)',
        border: '1px solid rgba(255, 255, 255, 0.08)'
      }}>
        <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>Pick</span>
        <select value={pickColor} onChange={(e) => setPickColor(e.target.value)}
          style={selectStyle}>
          {COLORS.map(c => <option key={c} value={c}>{cap(c)}</option>)}
        </select>
        <ArrowUp size={14} color="var(--text-tertiary)" />
        <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>place on</span>
        <select value={placeColor} onChange={(e) => setPlaceColor(e.target.value)}
          style={selectStyle}>
          {COLORS.map(c => <option key={c} value={c}>{cap(c)}</option>)}
        </select>
        <button
          className="apple-button apple-button-primary"
          disabled={pickColor === placeColor}
          onPointerDown={() => dispatch(
            `pick up the ${pickColor} block and place it on top of the ${placeColor} block`)}
          style={{ marginLeft: 'auto', fontSize: '13px', padding: '9px 16px',
                   opacity: pickColor === placeColor ? 0.4 : 1 }}
        >
          Go
        </button>
      </div>

      {/* Custom prompt */}
      <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '10px' }}>
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

const selectStyle: React.CSSProperties = {
  padding: '8px 10px',
  borderRadius: '8px',
  background: 'rgba(0, 0, 0, 0.4)',
  border: '1px solid rgba(255, 255, 255, 0.15)',
  color: '#fff',
  fontSize: '13px',
  outline: 'none',
  fontFamily: 'inherit'
};

const cap = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);
