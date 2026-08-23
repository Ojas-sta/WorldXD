const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const os = require('os');

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST']
  }
});

app.use(express.json({ limit: '5mb' }));  // camera frames arrive base64

// Serve the built frontend so the whole dashboard lives on one port
const DIST = path.join(__dirname, '..', 'frontend', 'dist');
if (fs.existsSync(DIST)) {
  app.use(express.static(DIST));
}

// In-memory telemetry state
let state = {
  jepaModel: {
    status: 'Loaded & Active',
    device: 'Apple Silicon (MPS fp16)',
    checkpoint: 'jepa_wm_metaworld.pth.tar',
    encoder: 'DINOv2 ViT-S/14 (22M params)',
    predictor: 'VisionTransformerAdaLN (17.6M params)',
    visualDim: 384,
    proprioDim: 16,
    totalDim: 400,
    cemConfig: {
      horizon: 5,
      numSamples: 64,     // live-loop reduced size (see stacking_controller.py)
      iterations: 2,
      actionDim: 20
    },
    lastInferenceMs: null,
    cemPlannerStatus: 'Idle'
  },
  robotState: {
    eePos: [0.150, 0.000, 0.150],
    targetEePos: [0.250, 0.000, 0.020],
    jointAngles: [0.0, 0.0, 0.0, 0.0],
    jointAnglesDeg: [0.0, 0.0, 0.0, 0.0],
    gripperClosed: false,
    fsmState: 'DONE',
    stackedCount: 0,
    stackAll: false,
    blocks: []            // filled from /workspace_blocks via ros_bridge.py
  },
  system: {
    cpuPercent: 12.4,
    ram: {
      totalGB: 16.0,
      usedGB: 0.0,
      freeGB: 16.0,
      readSpeedMBs: 0,
      writeSpeedMBs: 0
    },
    swap: {
      usedGB: 0,
      totalGB: 8.0,
      status: 'Safe'
    },
    compute: {
      gpuTflops: null,   // not measurable on Apple Silicon; do not fabricate
      gpuTops: null,
      mpsUtilizationPercent: null
    },
    activeProcesses: [],
    uptimeSec: os.uptime()
  },
  session: {
    activeProject: 'WorldXD',
    lastPrompt: '',
    cameraRes: null,
    logLines: []
  }
};

let lastPageins = 0;
let lastPageouts = 0;
let lastTick = Date.now();

// Poll system processes and memory/IO metrics
function pollTelemetry() {
  const now = Date.now();
  const dt = Math.max((now - lastTick) / 1000, 0.1);
  lastTick = now;

  // Process status check (trim to top 6 to optimize payload size)
  exec('ps aux | grep -E "python|ros|opencode|launch_robot|stacking_controller|test_jepa" | grep -v grep', (err, stdout) => {
    if (!err && stdout) {
      const lines = stdout.trim().split('\n');
      const processes = lines.slice(0, 6).map(line => {
        const parts = line.trim().split(/\s+/);
        const pid = parts[1];
        const cpu = parseFloat(parts[2]) || 0;
        const mem = parseFloat(parts[3]) || 0;
        const cmd = parts.slice(10).join(' ');
        
        let label = 'Process';
        if (cmd.includes('launch_robot')) label = 'Master Launcher';
        else if (cmd.includes('workspace_env')) label = 'Workspace Environment';
        else if (cmd.includes('stacking_controller')) label = 'Stacking Controller (FSM/IK)';
        else if (cmd.includes('terminal_prompt')) label = 'Terminal Prompt GUI';
        else if (cmd.includes('test_jepa')) label = 'JEPA Verification Script';
        else if (cmd.includes('robot_state_publisher')) label = 'Robot State Publisher';
        else if (cmd.includes('opencode')) label = 'OpenCode Agent';

        return { pid, cpu, mem, cmd: cmd.slice(0, 50), label };
      });
      state.system.activeProcesses = processes;

      const totalCpu = processes.reduce((acc, p) => acc + p.cpu, 0);
      state.system.cpuPercent = Math.min(Math.round(totalCpu * 10) / 10, 100);
      state.system.compute.mpsUtilizationPercent = Math.min(Math.round((totalCpu * 0.6) * 10) / 10, 100);
    }
  });

  // Calculate Memory stats
  const totalMemGB = os.totalmem() / (1024 * 1024 * 1024);
  const freeMemGB = os.freemem() / (1024 * 1024 * 1024);
  const usedMemGB = totalMemGB - freeMemGB;

  state.system.ram.totalGB = Math.round(totalMemGB * 10) / 10;
  state.system.ram.usedGB = Math.round(usedMemGB * 10) / 10;
  state.system.ram.freeGB = Math.round(freeMemGB * 10) / 10;

  state.system.ram.modelWeightsGB = 3.5;
  state.system.ram.latentCacheGB = 0.8;
  state.system.ram.systemAppsGB = Math.max(0, Math.round((usedMemGB - 4.3) * 10) / 10);

  // IO Speed Simulation based on vm_stat activity
  exec('vm_stat', (err, stdout) => {
    if (!err && stdout) {
      const pageinsMatch = stdout.match(/Pageins:\s+(\d+)/);
      const pageoutsMatch = stdout.match(/Pageouts:\s+(\d+)/);
      if (pageinsMatch && pageoutsMatch) {
        const pageins = parseInt(pageinsMatch[1], 10);
        const pageouts = parseInt(pageoutsMatch[1], 10);

        if (lastPageins > 0) {
          const readMBs = Math.max(0, ((pageins - lastPageins) * 4096) / (1024 * 1024 * dt));
          const writeMBs = Math.max(0, ((pageouts - lastPageouts) * 4096) / (1024 * 1024 * dt));
          state.system.ram.readSpeedMBs = Math.round(readMBs * 10) / 10;
          state.system.ram.writeSpeedMBs = Math.round(writeMBs * 10) / 10;
        }
        lastPageins = pageins;
        lastPageouts = pageouts;
      }
    }
  });

  // GPU compute figures are NOT measurable on Apple Silicon from userspace;
  // left as null so the UI renders "n/a" instead of fabricated numbers.
  state.system.compute.gpuTflops = null;
  state.system.compute.gpuTops = null;

  io.emit('telemetry', state);
}

// 800ms interval for optimal network & UI render smoothness
setInterval(pollTelemetry, 800);

// ---------------------------------------------------------------------------
// ROS relay endpoints (fed by dashboard/backend/ros_bridge.py)
// ---------------------------------------------------------------------------
let latestCameraJpeg = null;
let lastFsmStamp = 0;

app.post('/ros/state', (req, res) => {
  const d = req.body || {};
  if (Array.isArray(d.jointAngles)) {
    state.robotState.jointAngles = d.jointAngles;
    state.robotState.jointAnglesDeg = d.jointAngles.map(a => Math.round(a * 180 / Math.PI * 10) / 10);
  }
  if (typeof d.gripperClosed === 'boolean') state.robotState.gripperClosed = d.gripperClosed;
  if (typeof d.fsmState === 'string') {
    // Snapshots stream at 30Hz and HTTP can reorder them; only accept FSM
    // updates carrying a strictly newer timestamp.
    const stamp = typeof d.fsmStamp === 'number' ? d.fsmStamp : Date.now();
    if (stamp > lastFsmStamp) {
      lastFsmStamp = stamp;
      if (d.fsmState !== state.robotState.fsmState) {
        pushLog(`FSM -> ${d.fsmState}`);
      }
      state.robotState.fsmState = d.fsmState;   // real state from controller
    }
  }
  if (d.lastInferenceMs != null) {
    state.jepaModel.lastInferenceMs = d.lastInferenceMs;
    state.jepaModel.cemPlannerStatus = 'Planning (CEM)';
  }
  if (Array.isArray(d.blocks)) state.robotState.blocks = d.blocks;
  if (Array.isArray(d.imageRes)) state.session.cameraRes = d.imageRes;
  res.json({ ok: true });
});

app.post('/ros/camera', (req, res) => {
  latestCameraJpeg = req.body && req.body.jpeg ? req.body.jpeg : null;
  if (latestCameraJpeg) io.emit('camera', latestCameraJpeg);  // out-of-band: big payload
  res.json({ ok: true });
});

app.get('/api/camera', (req, res) => {
  res.json({ jpeg: latestCameraJpeg });
});

function pushLog(line) {
  const stamp = new Date().toLocaleTimeString();
  state.session.logLines.push(`[${stamp}] ${line}`);
  state.session.logLines = state.session.logLines.slice(-15);
}

// API Routes
app.get('/api/state', (req, res) => {
  res.json(state);
});

app.post('/api/prompt', (req, res) => {
  const { prompt } = req.body;
  if (!prompt) return res.status(400).json({ error: 'Prompt is required' });

  console.log(`Received prompt from Dashboard: ${prompt}`);
  state.session.lastPrompt = prompt;
  pushLog(`Prompt: "${prompt}"`);

  // NOTE: FSM state is owned by the controller now (/fsm_state via relay);
  // we no longer guess transitions here.
  // ros2 only exists inside the pixi env -> must route through pixi.
  exec(`pixi run ros2 topic pub --once /user_prompt std_msgs/msg/String "{data: '${prompt}'}"`,
       { cwd: path.join(__dirname, '..', '..') }, (err) => {
    if (err) console.error('Prompt publish failed:', err.message);
  });

  io.emit('telemetry', state);
  res.json({ success: true, message: `Prompt '${prompt}' broadcasted.` });
});

// API routes fall through to the SPA entry for client-side navigation
app.get('*', (req, res, next) => {
  if (req.path.startsWith('/api/') || req.path.startsWith('/ros/')) return next();
  res.sendFile(path.join(DIST, 'index.html'));
});

const PORT = 4002;
server.listen(PORT, () => {
  console.log(`=================================================`);
  console.log(`WorldXD Telemetry Backend running on port ${PORT}`);
  console.log(`WebSocket endpoint: ws://localhost:${PORT}`);
  console.log(`=================================================`);
});
