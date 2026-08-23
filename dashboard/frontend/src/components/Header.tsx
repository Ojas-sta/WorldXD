import React from 'react';
import { Cpu, Activity, ShieldCheck, Zap } from 'lucide-react';
import { TelemetryState } from '../types';

interface Props {
  connected: boolean;
  state: TelemetryState | null;
}

export const Header: React.FC<Props> = ({ connected, state }) => {
  return (
    <header className="apple-glass" style={{ padding: '16px 28px', marginBottom: '24px', display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
        <div style={{
          width: '42px',
          height: '42px',
          borderRadius: '12px',
          background: 'linear-gradient(135deg, #0A84FF 0%, #BF5AF2 100%)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          boxShadow: '0 4px 16px rgba(10, 132, 255, 0.4)'
        }}>
          <Zap size={22} color="#fff" />
        </div>
        <div>
          <h1 className="display-title">WorldXD Telemetry</h1>
          <p className="sub-title">Meta JEPA-WMS & Robot Arm Control Dashboard</p>
        </div>
      </div>

      <div style={{ display: 'flex', alignItems: 'center', gap: '20px' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', color: 'var(--text-secondary)' }}>
          <Activity size={16} color="var(--accent-teal)" />
          <span>FSM: <strong style={{ color: '#fff' }}>{state?.robotState.fsmState || 'IDLE'}</strong></span>
        </div>

        <div style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', color: 'var(--text-secondary)' }}>
          <Cpu size={16} color="var(--accent-purple)" />
          <span>Backend: <strong style={{ color: '#fff' }}>{state?.jepaModel.device || 'MPS (fp16)'}</strong></span>
        </div>

        <div className={`badge ${connected ? 'badge-green' : 'badge-orange'}`}>
          <div className="pulse-dot" style={{ background: connected ? '#30D158' : '#FF9F0A' }} />
          <span>{connected ? 'LIVE TELEMETRY' : 'DISCONNECTED'}</span>
        </div>
      </div>
    </header>
  );
};
