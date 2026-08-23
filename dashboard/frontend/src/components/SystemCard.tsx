import React from 'react';
import { Cpu, HardDrive, Terminal, Zap, ArrowDown, ArrowUp } from 'lucide-react';
import { motion } from 'framer-motion';
import { SystemMetrics } from '../types';

interface Props {
  system: SystemMetrics;
}

export const SystemCard: React.FC<Props> = ({ system }) => {
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
  const compute = system.compute || { gpuTflops: 2.8, gpuTops: 11.4, mpsUtilizationPercent: 48.0 };

  // Calculate circular gauge stroke dashoffsets (R = 36, Circ = 226)
  const calcDash = (val: number, max: number) => {
    const pct = Math.min(100, Math.max(0, (val / max) * 100));
    return 226 - (pct / 100) * 226;
  };

  const springConfig = { type: 'spring', damping: 25, stiffness: 90 };

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Cpu size={20} color="var(--accent-purple)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>System & Memory Telemetry</h2>
        </div>
        <span className="badge badge-purple">Apple Silicon MPS</span>
      </div>

      {/* IO Read & Write Speed Circular Instrument Cluster Gauges */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '12px', background: 'rgba(0, 0, 0, 0.3)', padding: '14px', borderRadius: '16px', border: '1px solid rgba(255, 255, 255, 0.08)' }}>
        {/* Read Gauge */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', position: 'relative' }}>
          <svg style={{ width: '84px', height: '84px', transform: 'rotate(-90deg)' }}>
            <circle cx="42" cy="42" r="34" stroke="rgba(255,255,255,0.08)" strokeWidth="6" fill="transparent" />
            <motion.circle
              cx="42"
              cy="42"
              r="34"
              stroke="#30D158"
              strokeWidth="6"
              strokeDasharray={213}
              animate={{ strokeDashoffset: calcDash(ram.readSpeedMBs, 100) }}
              strokeLinecap="round"
              fill="transparent"
              transition={springConfig}
            />
          </svg>
          <div style={{ position: 'absolute', top: '24px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <span style={{ fontSize: '14px', fontWeight: 700, fontFamily: 'monospace', color: '#30D158' }}>
              {ram.readSpeedMBs.toFixed(1)}
            </span>
            <span style={{ fontSize: '9px', color: 'var(--text-tertiary)' }}>MB/s</span>
          </div>
          <span style={{ fontSize: '11px', fontWeight: 600, color: 'var(--text-secondary)', marginTop: '4px', display: 'flex', alignItems: 'center', gap: '4px' }}>
            <ArrowDown size={11} color="#30D158" /> IO Read
          </span>
        </div>

        {/* Write Gauge */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', position: 'relative' }}>
          <svg style={{ width: '84px', height: '84px', transform: 'rotate(-90deg)' }}>
            <circle cx="42" cy="42" r="34" stroke="rgba(255,255,255,0.08)" strokeWidth="6" fill="transparent" />
            <motion.circle
              cx="42"
              cy="42"
              r="34"
              stroke="#0A84FF"
              strokeWidth="6"
              strokeDasharray={213}
              animate={{ strokeDashoffset: calcDash(ram.writeSpeedMBs, 100) }}
              strokeLinecap="round"
              fill="transparent"
              transition={springConfig}
            />
          </svg>
          <div style={{ position: 'absolute', top: '24px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <span style={{ fontSize: '14px', fontWeight: 700, fontFamily: 'monospace', color: '#0A84FF' }}>
              {ram.writeSpeedMBs.toFixed(1)}
            </span>
            <span style={{ fontSize: '9px', color: 'var(--text-tertiary)' }}>MB/s</span>
          </div>
          <span style={{ fontSize: '11px', fontWeight: 600, color: 'var(--text-secondary)', marginTop: '4px', display: 'flex', alignItems: 'center', gap: '4px' }}>
            <ArrowUp size={11} color="#0A84FF" /> IO Write
          </span>
        </div>
      </div>

      {/* Unified RAM Breakdown Bars */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: '10px', background: 'rgba(255, 255, 255, 0.02)', padding: '14px', borderRadius: '14px', border: '1px solid rgba(255, 255, 255, 0.06)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px' }}>
          <span style={{ color: 'var(--text-secondary)', fontWeight: 500 }}>Unified RAM Allocation</span>
          <span style={{ fontWeight: 600, fontFamily: 'monospace' }}>{ram.usedGB} GB / {ram.totalGB} GB</span>
        </div>

        {/* 1. Model Weights */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', marginBottom: '3px' }}>
            <span style={{ color: 'var(--accent-teal)' }}>JEPA Model Weights</span>
            <span>{ram.modelWeightsGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '6px', background: 'rgba(255,255,255,0.08)', borderRadius: '3px', overflow: 'hidden' }}>
            <motion.div style={{ background: 'var(--accent-teal)', height: '100%', borderRadius: '3px' }} animate={{ width: `${(ram.modelWeightsGB / ram.totalGB) * 100}%` }} transition={springConfig} />
          </div>
        </div>

        {/* 2. Latent Feature Cache */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', marginBottom: '3px' }}>
            <span style={{ color: 'var(--accent-purple)' }}>Latent Feature Cache</span>
            <span>{ram.latentCacheGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '6px', background: 'rgba(255,255,255,0.08)', borderRadius: '3px', overflow: 'hidden' }}>
            <motion.div style={{ background: 'var(--accent-purple)', height: '100%', borderRadius: '3px' }} animate={{ width: `${(ram.latentCacheGB / ram.totalGB) * 100}%` }} transition={springConfig} />
          </div>
        </div>

        {/* 3. System & ROS2 Apps */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', marginBottom: '3px' }}>
            <span style={{ color: 'var(--accent-orange)' }}>System & ROS2 Subsystems</span>
            <span>{ram.systemAppsGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '6px', background: 'rgba(255,255,255,0.08)', borderRadius: '3px', overflow: 'hidden' }}>
            <motion.div style={{ background: 'var(--accent-orange)', height: '100%', borderRadius: '3px' }} animate={{ width: `${(ram.systemAppsGB / ram.totalGB) * 100}%` }} transition={springConfig} />
          </div>
        </div>

        {/* 4. Free Headroom */}
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', marginBottom: '3px' }}>
            <span style={{ color: 'var(--text-tertiary)' }}>Free Headroom</span>
            <span>{ram.freeGB} GB</span>
          </div>
          <div style={{ width: '100%', height: '6px', background: 'rgba(255,255,255,0.08)', borderRadius: '3px', overflow: 'hidden' }}>
            <motion.div style={{ background: 'rgba(255,255,255,0.3)', height: '100%', borderRadius: '3px' }} animate={{ width: `${(ram.freeGB / ram.totalGB) * 100}%` }} transition={springConfig} />
          </div>
        </div>
      </div>

      {/* SSD Swap Status Bar */}
      <div style={{ background: 'rgba(0,0,0,0.25)', padding: '10px 14px', borderRadius: '12px', border: '1px solid rgba(255,255,255,0.06)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: '11px', marginBottom: '6px' }}>
          <span style={{ color: 'var(--text-secondary)', display: 'flex', alignItems: 'center', gap: '6px' }}>
            <HardDrive size={13} color="var(--accent-blue)" /> SSD Swap Buffer
          </span>
          <span style={{ fontWeight: 600, color: 'var(--accent-blue)' }}>{swap.usedGB.toFixed(2)} GB / {swap.totalGB} GB</span>
        </div>
        <div style={{ width: '100%', height: '5px', background: 'rgba(255,255,255,0.08)', borderRadius: '3px', overflow: 'hidden' }}>
          <motion.div style={{ background: 'var(--accent-blue)', height: '100%', borderRadius: '3px' }} animate={{ width: `${(swap.usedGB / swap.totalGB) * 100}%` }} transition={springConfig} />
        </div>
      </div>

      {/* Subsystem Process List */}
      <div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-secondary)', marginBottom: '8px' }}>
          <Terminal size={14} color="var(--accent-teal)" /> Active Subsystem Processes
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px', maxHeight: '130px', overflowY: 'auto' }}>
          {system.activeProcesses.length > 0 ? (
            system.activeProcesses.map((proc, idx) => (
              <div key={idx} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', padding: '6px 10px', background: 'rgba(255, 255, 255, 0.03)', borderRadius: '8px', border: '1px solid rgba(255,255,255,0.05)', fontSize: '11px' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                  <span style={{ width: '6px', height: '6px', borderRadius: '50%', background: 'var(--accent-teal)' }} />
                  <span style={{ fontWeight: 500 }}>{proc.label}</span>
                </div>
                <div style={{ fontSize: '10px', color: 'var(--text-secondary)', fontFamily: 'monospace' }}>
                  {proc.cpu.toFixed(1)}% CPU
                </div>
              </div>
            ))
          ) : (
            <div style={{ fontSize: '11px', color: 'var(--text-tertiary)', padding: '10px', textAlign: 'center' }}>
              No background workspace processes running
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
