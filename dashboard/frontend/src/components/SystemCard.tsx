import React from 'react';
import { Cpu, HardDrive, Terminal, ArrowDown, ArrowUp } from 'lucide-react';
import { SystemMetrics } from '../types';

interface Props {
  system: SystemMetrics;
}

export const SystemCard: React.FC<Props> = React.memo(({ system }) => {
  const ram = system.ram || {
    totalGB: 16.0,
    usedGB: 9.4,
    freeGB: 6.6,
    modelWeightsGB: 3.5,
    latentCacheGB: 0.8,
    systemAppsGB: 5.1,
    readSpeedMBs: 42.5,
    writeSpeedMBs: 18.2
  };

  const swap = system.swap || { usedGB: 1.1, totalGB: 8.0, status: 'Safe' };

  // Calculate circular gauge stroke dashoffset (R = 34, Circ = 213)
  const calcDash = (val: number, max: number) => {
    const pct = Math.min(100, Math.max(0, (val / max) * 100));
    return 213 - (pct / 100) * 213;
  };

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Cpu size={18} color="var(--accent-purple)" />
          <h2 style={{ fontSize: '15px', fontWeight: 600 }}>System & Memory Telemetry</h2>
        </div>
        <span className="badge badge-purple">Apple Silicon MPS</span>
      </div>

      {/* IO Read & Write Speed Circular Instrument Cluster Gauges */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '12px', background: 'rgba(0, 0, 0, 0.25)', padding: '12px', borderRadius: '14px', border: '1px solid rgba(255, 255, 255, 0.06)' }}>
        {/* Read Gauge */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', position: 'relative' }}>
          <svg style={{ width: '76px', height: '76px', transform: 'rotate(-90deg)' }}>
            <circle cx="38" cy="38" r="34" stroke="rgba(255,255,255,0.06)" strokeWidth="5" fill="transparent" />
            <circle
              cx="38"
              cy="38"
              r="34"
              stroke="#30D158"
              strokeWidth="5"
              strokeDasharray={213}
              strokeDashoffset={calcDash(ram.readSpeedMBs, 100)}
              strokeLinecap="round"
              fill="transparent"
              style={{ transition: 'stroke-dashoffset 0.3s ease-out' }}
            />
          </svg>
          <div style={{ position: 'absolute', top: '22px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <span style={{ fontSize: '13px', fontWeight: 700, fontFamily: 'monospace', color: '#30D158' }}>
              {ram.readSpeedMBs.toFixed(1)}
            </span>
            <span style={{ fontSize: '9px', color: 'var(--text-tertiary)' }}>MB/s</span>
          </div>
          <span style={{ fontSize: '11px', fontWeight: 600, color: 'var(--text-secondary)', marginTop: '4px', display: 'flex', alignItems: 'center', gap: '3px' }}>
            <ArrowDown size={11} color="#30D158" /> IO Read
          </span>
        </div>

        {/* Write Gauge */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', position: 'relative' }}>
          <svg style={{ width: '76px', height: '76px', transform: 'rotate(-90deg)' }}>
            <circle cx="38" cy="38" r="34" stroke="rgba(255,255,255,0.06)" strokeWidth="5" fill="transparent" />
            <circle
              cx="38"
              cy="38"
              r="34"
              stroke="#0A84FF"
              strokeWidth="5"
              strokeDasharray={213}
              strokeDashoffset={calcDash(ram.writeSpeedMBs, 100)}
              strokeLinecap="round"
              fill="transparent"
              style={{ transition: 'stroke-dashoffset 0.3s ease-out' }}
            />
          </svg>
          <div style={{ position: 'absolute', top: '22px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <span style={{ fontSize: '13px', fontWeight: 700, fontFamily: 'monospace', color: '#0A84FF' }}>
              {ram.writeSpeedMBs.toFixed(1)}
            </span>
            <span style={{ fontSize: '9px', color: 'var(--text-tertiary)' }}>MB/s</span>
          </div>
          <span style={{ fontSize: '11px', fontWeight: 600, color: 'var(--text-secondary)', marginTop: '4px', display: 'flex', alignItems: 'center', gap: '3px' }}>
            <ArrowUp size={11} color="#0A84FF" /> IO Write
          </span>
        </div>
      </div>

      {/* Unified RAM Breakdown Bars */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', background: 'rgba(255, 255, 255, 0.02)', padding: '12px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.05)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px' }}>
          <span style={{ color: 'var(--text-secondary)', fontWeight: 500 }}>Unified RAM Allocation</span>
          <span style={{ fontWeight: 600, fontFamily: 'monospace' }}>{ram.usedGB} GB / {ram.totalGB} GB</span>
        </div>

        {/* 1. Model Weights */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', marginBottom: '2px' }}>
            <span style={{ color: 'var(--accent-teal)' }}>JEPA Model Weights</span>
            <span>{ram.modelWeightsGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '5px', background: 'rgba(255,255,255,0.06)', borderRadius: '3px', overflow: 'hidden' }}>
            <div style={{ width: `${(ram.modelWeightsGB / ram.totalGB) * 100}%`, background: 'var(--accent-teal)', height: '100%', borderRadius: '3px', transition: 'width 0.3s ease' }} />
          </div>
        </div>

        {/* 2. Latent Feature Cache */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', marginBottom: '2px' }}>
            <span style={{ color: 'var(--accent-purple)' }}>Latent Feature Cache</span>
            <span>{ram.latentCacheGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '5px', background: 'rgba(255,255,255,0.06)', borderRadius: '3px', overflow: 'hidden' }}>
            <div style={{ width: `${(ram.latentCacheGB / ram.totalGB) * 100}%`, background: 'var(--accent-purple)', height: '100%', borderRadius: '3px', transition: 'width 0.3s ease' }} />
          </div>
        </div>

        {/* 3. System & ROS2 Apps */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', marginBottom: '2px' }}>
            <span style={{ color: 'var(--accent-orange)' }}>System & ROS2 Subsystems</span>
            <span>{ram.systemAppsGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '5px', background: 'rgba(255,255,255,0.06)', borderRadius: '3px', overflow: 'hidden' }}>
            <div style={{ width: `${(ram.systemAppsGB / ram.totalGB) * 100}%`, background: 'var(--accent-orange)', height: '100%', borderRadius: '3px', transition: 'width 0.3s ease' }} />
          </div>
        </div>

        {/* 4. Free Headroom */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', marginBottom: '2px' }}>
            <span style={{ color: 'var(--text-tertiary)' }}>Free Headroom</span>
            <span>{ram.freeGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '5px', background: 'rgba(255,255,255,0.06)', borderRadius: '3px', overflow: 'hidden' }}>
            <div style={{ width: `${(ram.freeGB / ram.totalGB) * 100}%`, background: 'rgba(255,255,255,0.25)', height: '100%', borderRadius: '3px', transition: 'width 0.3s ease' }} />
          </div>
        </div>
      </div>

      {/* SSD Swap Status Bar */}
      <div style={{ background: 'rgba(0,0,0,0.2)', padding: '10px 12px', borderRadius: '10px', border: '1px solid rgba(255,255,255,0.05)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: '11px', marginBottom: '4px' }}>
          <span style={{ color: 'var(--text-secondary)', display: 'flex', alignItems: 'center', gap: '5px' }}>
            <HardDrive size={12} color="var(--accent-blue)" /> SSD Swap Buffer
          </span>
          <span style={{ fontWeight: 600, color: 'var(--accent-blue)' }}>{swap.usedGB.toFixed(2)} GB / {swap.totalGB} GB</span>
        </div>
        <div style={{ width: '100%', height: '4px', background: 'rgba(255,255,255,0.06)', borderRadius: '2px', overflow: 'hidden' }}>
          <div style={{ width: `${(swap.usedGB / swap.totalGB) * 100}%`, background: 'var(--accent-blue)', height: '100%', borderRadius: '2px', transition: 'width 0.3s ease' }} />
        </div>
      </div>

      {/* Subsystem Process List */}
      <div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '11px', color: 'var(--text-secondary)', marginBottom: '6px' }}>
          <Terminal size={13} color="var(--accent-teal)" /> Active Subsystem Processes
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: '5px', maxHeight: '110px', overflowY: 'auto' }}>
          {system.activeProcesses.length > 0 ? (
            system.activeProcesses.slice(0, 6).map((proc, idx) => (
              <div key={idx} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', padding: '5px 8px', background: 'rgba(255, 255, 255, 0.02)', borderRadius: '6px', border: '1px solid rgba(255,255,255,0.04)', fontSize: '10px' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
                  <span style={{ width: '5px', height: '5px', borderRadius: '50%', background: 'var(--accent-teal)' }} />
                  <span style={{ fontWeight: 500 }}>{proc.label}</span>
                </div>
                <div style={{ fontSize: '10px', color: 'var(--text-secondary)', fontFamily: 'monospace' }}>
                  {proc.cpu.toFixed(1)}% CPU
                </div>
              </div>
            ))
          ) : (
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)', padding: '8px', textAlign: 'center' }}>
              No background workspace processes running
            </div>
          )}
        </div>
      </div>
    </div>
  );
});
