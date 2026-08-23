import React, { useEffect, useRef } from 'react';
import { Terminal, Scroll } from 'lucide-react';

interface Props {
  logs: string[];
}

export const LogStream: React.FC<Props> = ({ logs }) => {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (containerRef.current) {
      containerRef.current.scrollTop = containerRef.current.scrollHeight;
    }
  }, [logs]);

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Terminal size={18} color="var(--accent-teal)" />
          <h2 style={{ fontSize: '15px', fontWeight: 600 }}>Task Execution Log Stream</h2>
        </div>
        <span className="badge badge-blue">Stdout Stream</span>
      </div>

      <div
        ref={containerRef}
        style={{
          background: '#090a0f',
          borderRadius: '10px',
          padding: '12px',
          fontFamily: 'monospace',
          fontSize: '11px',
          lineHeight: '1.6',
          color: '#a0aab8',
          maxHeight: '160px',
          overflowY: 'auto',
          border: '1px solid rgba(255, 255, 255, 0.08)'
        }}
      >
        {logs.length > 0 ? (
          logs.map((line, i) => (
            <div key={i} style={{ whiteSpace: 'pre-wrap', wordBreak: 'break-all' }}>
              <span style={{ color: 'var(--text-tertiary)', marginRight: '8px' }}>[{i + 1}]</span>
              {line}
            </div>
          ))
        ) : (
          <div style={{ color: 'var(--text-tertiary)', fontStyle: 'italic' }}>
            System logs initialized. Waiting for task output...
          </div>
        )}
      </div>
    </div>
  );
};
