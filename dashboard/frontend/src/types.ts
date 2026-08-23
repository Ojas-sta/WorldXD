export interface CEMConfig {
  horizon: number;
  numSamples: number;
  iterations: number;
  numElites: number;
  actionDim: number;
}

export interface JEPAModelState {
  status: string;
  device: string;
  checkpoint: string;
  encoder: string;
  predictor: string;
  visualDim: number;
  proprioDim: number;
  totalDim: number;
  cemConfig: CEMConfig;
  lastInferenceMs: number;
  cemPlannerStatus: string;
}

export interface BlockTarget {
  id: number;
  color: string;
  pos: [number, number, number];
  status: string;
}

export interface RobotState {
  eePos: [number, number, number];
  targetEePos: [number, number, number];
  jointAngles: [number, number, number, number];
  jointAnglesDeg: [number, number, number, number];
  gripperClosed: boolean;
  fsmState: string;
  stackedCount: number;
  stackAll: boolean;
  blocks: BlockTarget[];
}

export interface ProcessItem {
  pid: string;
  cpu: number;
  mem: number;
  cmd: string;
  label: string;
}

export interface RamDetails {
  totalGB: number;
  usedGB: number;
  freeGB: number;
  modelWeightsGB: number;
  latentCacheGB: number;
  systemAppsGB: number;
  readSpeedMBs: number;
  writeSpeedMBs: number;
}

export interface SwapDetails {
  usedGB: number;
  totalGB: number;
  status: string;
}

export interface ComputeMetrics {
  gpuTflops: number;
  gpuTops: number;
  mpsUtilizationPercent: number;
}

export interface SystemMetrics {
  cpuPercent: number;
  ram: RamDetails;
  swap: SwapDetails;
  compute: ComputeMetrics;
  activeProcesses: ProcessItem[];
  uptimeSec: number;
}

export interface SessionData {
  activeProject: string;
  lastPrompt: string;
  opencodeStatus: string;
  logLines: string[];
}

export interface TelemetryState {
  jepaModel: JEPAModelState;
  robotState: RobotState;
  system: SystemMetrics;
  session: SessionData;
}
