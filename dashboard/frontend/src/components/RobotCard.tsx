import React from 'react';
import { Bot, Target, Compass, Lock, Unlock } from 'lucide-react';
import { RobotState } from '../types';

interface Props {
  robot: RobotState;
}

export const RobotCard: React.FC<Props> = ({ robot }) => {
  const formatNum = (n: number) => (n >= 0 ? `+${n.toFixed(3)}` : n.toFixed(3));

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Bot size={20} color="var(--accent-blue)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>EEZYbotARM MK2 State</h2>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
          <span className="badge badge-purple">{robot.fsmState}</span>
          <div style={{ padding: '4px 8px', borderRadius: '8px', background: robot.gripperClosed ? 'rgba(255,69,58,0.2)' : 'rgba(48,209,88,0.2)', color: robot.gripperClosed ? '#FF6961' : '#30D158', fontSize: '11px', fontWeight: 600, display: 'flex', alignItems: 'center', gap: '4px' }}>
            {robot.gripperClosed ? <Lock size={12} /> : <Unlock size={12} />}
            {robot.gripperClosed ? 'CLOSED' : 'OPEN'}
          </div>
        </div>
      </div>

      {/* End Effector Position Gauges */}
      <div style={{ background: 'rgba(0, 0, 0, 0.25)', padding: '14px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.08)' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-secondary)', marginBottom: '10px' }}>
          <Target size={14} color="var(--accent-teal)" /> End-Effector Spatial Coordinates (x, y, z)
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '10px', textAlign: 'center' }}>
          <div style={{ background: 'rgba(255,255,255,0.04)', padding: '10px', borderRadius: '10px' }}>
            <div style={{ fontSize: '11px', color: 'var(--accent-teal)' }}>X Axis</div>
            <div style={{ fontSize: '16px', fontWeight: 600, fontFamily: 'monospace' }}>{formatNum(robot.eePos[0])}m</div>
          </div>
          <div style={{ background: 'rgba(255,255,255,0.04)', padding: '10px', borderRadius: '10px' }}>
            <div style={{ fontSize: '11px', color: 'var(--accent-blue)' }}>Y Axis</div>
            <div style={{ fontSize: '16px', fontWeight: 600, fontFamily: 'monospace' }}>{formatNum(robot.eePos[1])}m</div>
          </div>
          <div style={{ background: 'rgba(255,255,255,0.04)', padding: '10px', borderRadius: '10px' }}>
            <div style={{ fontSize: '11px', color: 'var(--accent-purple)' }}>Z Axis</div>
            <div style={{ fontSize: '16px', fontWeight: 600, fontFamily: 'monospace' }}>{formatNum(robot.eePos[2])}m</div>
          </div>
        </div>
      </div>

      {/* Joint Angles Table */}
      <div style={{ background: 'rgba(255, 255, 255, 0.03)', padding: '12px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.06)' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-secondary)', marginBottom: '8px' }}>
          <Compass size={14} color="var(--accent-orange)" /> 4-DOF Analytical IK Angles
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '6px', textAlign: 'center' }}>
          {robot.jointAnglesDeg.map((deg, idx) => (
            <div key={idx} style={{ padding: '6px', background: 'rgba(0,0,0,0.2)', borderRadius: '6px' }}>
              <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>J{idx + 1}</div>
              <div style={{ fontSize: '13px', fontWeight: 600 }}>{deg.toFixed(1)}°</div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
