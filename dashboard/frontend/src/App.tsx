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
import { CameraFeed } from './components/CameraFeed';
import { FsmPipeline } from './components/FsmPipeline';
import { TelemetryState, SystemMetrics } from './types';

const SOCKET_URL = 'http://localhost:4002';

// Real FSM states published by stacking_controller.py on /fsm_state
const FSM_ORDER = [
  'MANUAL', 'MOVE_ABOVE_BLOCK', 'DESCEND', 'CLOSE_GRIPPER', 'LIFT',
  'MOVE_ABOVE_STACK', 'PLACE', 'OPEN_GRIPPER', 'RETREAT'
];

export function App() {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryState | null>(null);
  const [socket, setSocket] = useState<ReturnType<typeof io> | null>(null);

  useEffect(() => {
    const s = io(SOCKET_URL, {
      transports: ['websocket', 'polling']
    });

    s.on('connect', () => {
      setConnected(true);
    });

    s.on('disconnect', () => {
      setConnected(false);
    });

    s.on('telemetry', (data: TelemetryState) => {
      setTelemetry(data);
    });

    setSocket(s);
    return () => {
      s.disconnect();
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
    cemConfig: { horizon: 5, numSamples: 64, iterations: 2, numElites: 8, actionDim: 20 },
    lastInferenceMs: null as unknown as number,
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
        {/* Left Column - Model, System Stats & Camera (4 cols) */}
        <div style={{ gridColumn: 'span 4', display: 'flex', flexDirection: 'column', gap: '20px' }}>
          <ModelCard model={defaultModel} />
          <CameraFeed socket={socket} resolution={telemetry?.session.cameraRes ?? null} />
          <CameraFeed socket={socket} resolution={null}
            event="goal_camera" title="JEPA Goal View"
            iconColor="var(--accent-teal)" note="rendered target · 1 fps" />
          <SystemCard system={defaultSystem} />
        </div>

        {/* Center/Right Column - Visualizer & Controls (8 cols) */}
        <div style={{ gridColumn: 'span 8', display: 'flex', flexDirection: 'column', gap: '20px' }}>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '20px' }}>
            <RobotCard robot={defaultRobot} />
            <Visualizer robot={defaultRobot} />
          </div>

          <FsmPipeline current={defaultRobot.fsmState} order={FSM_ORDER} />

          <ControlPanel onSendPrompt={handleSendPrompt} lastPrompt={telemetry?.session.lastPrompt || ''} />

          <LogStream logs={telemetry?.session.logLines || []} />
        </div>
      </motion.div>
    </div>
  );
}

export default App;
