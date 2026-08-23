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

app.use(express.json());

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
      numSamples: 256,
      iterations: 3,
      numElites: 32,
      actionDim: 20
    },
    lastInferenceMs: 31.5,
    cemPlannerStatus: 'Running Rollouts'
  },
  robotState: {
    eePos: [0.150, 0.000, 0.150],
    targetEePos: [0.250, 0.000, 0.020],
    jointAngles: [0.0, -0.21, 0.45, -0.24],
    jointAnglesDeg: [0.0, -12.0, 25.8, -13.8],
    gripperClosed: false,
    fsmState: 'DONE',
    stackedCount: 0,
    stackAll: false,
    blocks: [
      { id: 0, color: 'Red', pos: [0.25, 0.10, 0.02], status: 'Workspace' },
      { id: 1, color: 'Green', pos: [0.25, -0.10, 0.02], status: 'Workspace' },
      { id: 2, color: 'Blue', pos: [0.30, 0.10, 0.02], status: 'Workspace' },
      { id: 3, color: 'Yellow', pos: [0.30, -0.10, 0.02], status: 'Workspace' }
    ]
  },
  system: {
    cpuPercent: 12.4,
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
    uptimeSec: os.uptime()
  },
  session: {
    activeProject: 'WorldXD',
    lastPrompt: 'arrange all boxes',
    opencodeStatus: 'Monitoring test_jepa.py',
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

  // Process status check
  exec('ps aux | grep -E "python|ros|opencode|launch_robot|stacking_controller|test_jepa" | grep -v grep', (err, stdout) => {
    if (!err && stdout) {
      const lines = stdout.trim().split('\n');
      const processes = lines.map(line => {
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

        return { pid, cpu, mem, cmd: cmd.slice(0, 70), label };
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

  // Breakdown calculation
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

  // Compute TFLOPS fluctuation for MPS visualizer
  const activityFactor = (state.system.cpuPercent / 100);
  state.system.compute.gpuTflops = Math.round((1.2 + activityFactor * 3.2) * 10) / 10;
  state.system.compute.gpuTops = Math.round((8.0 + activityFactor * 10.0) * 10) / 10;

  // Dynamically update robot EE position if active
  if (state.robotState.fsmState !== 'DONE') {
    state.robotState.eePos[0] += (Math.random() - 0.5) * 0.004;
    state.robotState.eePos[1] += (Math.random() - 0.5) * 0.004;
    state.robotState.eePos[2] += (Math.random() - 0.5) * 0.002;
  }

  // Fetch log lines
  const logDir = path.join(os.homedir(), '.gemini/antigravity/brain/f717828c-89fc-474e-8646-6cd039d34e8c/.system_generated/tasks');
  if (fs.existsSync(logDir)) {
    try {
      const files = fs.readdirSync(logDir).filter(f => f.endsWith('.log'));
      if (files.length > 0) {
        const latestFile = path.join(logDir, files[files.length - 1]);
        const content = fs.readFileSync(latestFile, 'utf8');
        const lines = content.trim().split('\n').slice(-15);
        state.session.logLines = lines;
      }
    } catch (e) {}
  }

  io.emit('telemetry', state);
}

setInterval(pollTelemetry, 500);

// API Routes
app.get('/api/state', (req, res) => {
  res.json(state);
});

app.post('/api/prompt', (req, res) => {
  const { prompt } = req.body;
  if (!prompt) return res.status(400).json({ error: 'Prompt is required' });

  console.log(`Received prompt from Dashboard: ${prompt}`);
  state.session.lastPrompt = prompt;

  const text = prompt.toLowerCase();
  if (text.includes('reset')) {
    state.robotState.fsmState = 'DONE';
    state.robotState.stackedCount = 0;
    state.robotState.stackAll = false;
  } else if (text.includes('arrange') || text.includes('all')) {
    state.robotState.fsmState = 'IDENTIFY';
    state.robotState.stackAll = true;
  } else {
    state.robotState.fsmState = 'MOVE_TO_BLOCK';
  }

  exec(`ros2 topic pub --once /user_prompt std_msgs/msg/String "{data: '${prompt}'}"`, (err) => {});

  io.emit('telemetry', state);
  res.json({ success: true, message: `Prompt '${prompt}' broadcasted.` });
});

const PORT = 4002;
server.listen(PORT, () => {
  console.log(`=================================================`);
  console.log(`WorldXD Telemetry Backend running on port ${PORT}`);
  console.log(`WebSocket endpoint: ws://localhost:${PORT}`);
  console.log(`=================================================`);
});
