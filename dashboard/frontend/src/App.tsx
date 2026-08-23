import React, { useEffect, useState } from 'react';
import { io } from 'socket.io-client';
import { motion } from 'framer-motion';
import { Header } from './components/Header';
import { ModelCard } from './components/ModelCard';
import { RobotCard } from './components/RobotCard';
import { Visualizer } from './components/Visualizer';
import { SystemCard } from './components/SystemCard';
import { ControlPanel } from './components/ControlPanel';
import { LogStream } from './components/LogStream';
import { TelemetryState, SystemMetrics } from './types';

const SOCKET_URL = 'http://localhost:4002';

export function App() {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryState | null>(null);

  useEffect(() => {
    const socket = io(SOCKET_URL, {
      transports: ['websocket', 'polling']
    });

    socket.on('connect', () => {
      setConnected(true);
    });

    socket.on('disconnect', () => {
      setConnected(false);
    });

    socket.on('telemetry', (data: TelemetryState) => {
      setTelemetry(data);
    });

    return () => {
      socket.disconnect();
    };
  }, []);

  const handleSendPrompt = async (prompt: string) => {
    try {
      await fetch('http://localhost:4002/api/prompt', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ prompt })
      });
    } catch (e) {
      console.error('Failed to send prompt:', e);
    }
  };

  const defaultModel = telemetry?.jepaModel || {
    status: 'Active',
    device: 'Apple MPS float16',
    checkpoint: 'jepa_wm_metaworld.pth.tar',
    encoder: 'DINOv2 ViT-S/14',
    predictor: 'VisionTransformerAdaLN',
    visualDim: 384,
    proprioDim: 16,
    totalDim: 400,
    cemConfig: { horizon: 5, numSamples: 256, iterations: 3, numElites: 32, actionDim: 20 },
    lastInferenceMs: 31.5,
    cemPlannerStatus: 'Active'
  };

  const defaultRobot = telemetry?.robotState || {
    eePos: [0.15, 0.0, 0.15],
    targetEePos: [0.25, 0.0, 0.02],
    jointAngles: [0, -0.21, 0.45, -0.24],
    jointAnglesDeg: [0, -12, 25.8, -13.8],
    gripperClosed: false,
    fsmState: 'DONE',
    stackedCount: 0,
    stackAll: false,
    blocks: [
      { id: 0, color: 'Red', pos: [0.25, 0.1, 0.02], status: 'Workspace' },
      { id: 1, color: 'Green', pos: [0.25, -0.1, 0.02], status: 'Workspace' },
      { id: 2, color: 'Blue', pos: [0.3, 0.1, 0.02], status: 'Workspace' },
      { id: 3, color: 'Yellow', pos: [0.3, -0.1, 0.02], status: 'Workspace' }
    ]
  };

  const defaultSystem: SystemMetrics = telemetry?.system || {
    cpuPercent: 14.2,
    ram: {
      totalGB: 16.0,
      usedGB: 9.4,
      freeGB: 6.6,
      modelWeightsGB: 3.5,
      latentCacheGB: 0.8,
      systemAppsGB: 5.1,
      readSpeedMBs: 42.5,
      writeSpeedMBs: 18.2
    },
    swap: {
      usedGB: 1.1,
      totalGB: 8.0,
      status: 'Safe'
    },
    compute: {
      gpuTflops: 2.8,
      gpuTops: 11.4,
      mpsUtilizationPercent: 48.0
    },
    activeProcesses: [],
    uptimeSec: 3600
  };

  const springTransition = { type: 'spring', damping: 25, stiffness: 220 };

  return (
    <div style={{ maxWidth: '1400px', margin: '0 auto', padding: '24px 20px' }}>
      <Header connected={connected} state={telemetry} />

      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={springTransition}
        style={{ display: 'grid', gridTemplateColumns: 'repeat(12, 1fr)', gap: '20px' }}
      >
        {/* Left Column - Model & System Stats (4 cols) */}
        <div style={{ gridColumn: 'span 4', display: 'flex', flexDirection: 'column', gap: '20px' }}>
          <ModelCard model={defaultModel} />
          <SystemCard system={defaultSystem} />
        </div>

        {/* Center/Right Column - Visualizer & Controls (8 cols) */}
        <div style={{ gridColumn: 'span 8', display: 'flex', flexDirection: 'column', gap: '20px' }}>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '20px' }}>
            <RobotCard robot={defaultRobot} />
            <Visualizer robot={defaultRobot} />
          </div>

          <ControlPanel onSendPrompt={handleSendPrompt} lastPrompt={telemetry?.session.lastPrompt || 'arrange all boxes'} />

          <LogStream logs={telemetry?.session.logLines || []} />
        </div>
      </motion.div>
    </div>
  );
}

export default App;
