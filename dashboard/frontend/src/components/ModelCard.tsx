import React from 'react';
import { Brain, Layers, Sliders, HardDrive, CheckCircle2 } from 'lucide-react';
import { JEPAModelState } from '../types';

interface Props {
  model: JEPAModelState;
}

export const ModelCard: React.FC<Props> = ({ model }) => {
  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Brain size={20} color="var(--accent-teal)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>JEPA-WMS Neural Engine</h2>
        </div>
        <span className="badge badge-blue">MPS Acceleration</span>
      </div>

      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '12px' }}>
        <div style={{ background: 'rgba(255, 255, 255, 0.04)', padding: '12px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.07)' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '11px', color: 'var(--text-secondary)' }}>
            <Layers size={13} /> Visual Encoder
          </div>
          <div style={{ fontSize: '13px', fontWeight: 600, marginTop: '4px' }}>DINOv2 ViT-S/14</div>
          <div style={{ fontSize: '11px', color: 'var(--text-tertiary)' }}>22.05M params • Frozen</div>
        </div>

        <div style={{ background: 'rgba(255, 255, 255, 0.04)', padding: '12px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.07)' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '11px', color: 'var(--text-secondary)' }}>
            <Sliders size={13} /> Predictor
          </div>
          <div style={{ fontSize: '13px', fontWeight: 600, marginTop: '4px' }}>ViT AdaLN (6 Blocks)</div>
          <div style={{ fontSize: '11px', color: 'var(--text-tertiary)' }}>17.63M params • Trainable</div>
        </div>
      </div>

      <div style={{ background: 'rgba(0, 0, 0, 0.25)', padding: '14px', borderRadius: '12px', border: '1px solid rgba(255, 255, 255, 0.08)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '8px', fontSize: '12px' }}>
          <span style={{ color: 'var(--text-secondary)' }}>Latent Dimension Alignment</span>
          <span style={{ fontWeight: 600, color: 'var(--accent-teal)' }}>{model.totalDim} Dims</span>
        </div>
        <div style={{ width: '100%', height: '8px', background: 'rgba(255, 255, 255, 0.1)', borderRadius: '4px', overflow: 'hidden', display: 'flex' }}>
          <div style={{ width: '96%', background: 'var(--accent-blue)', height: '100%' }} title="Visual Latent (384-dim)" />
          <div style={{ width: '4%', background: 'var(--accent-purple)', height: '100%' }} title="Proprio Latent (16-dim)" />
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginTop: '6px', fontSize: '11px', color: 'var(--text-tertiary)' }}>
          <span>384 Visual</span>
          <span>16 Proprio Feature Concat</span>
        </div>
      </div>

      <div style={{ borderTop: '1px solid rgba(255, 255, 255, 0.08)', paddingTop: '12px' }}>
        <div style={{ fontSize: '12px', fontWeight: 600, marginBottom: '8px', color: 'var(--text-secondary)' }}>
          Cross-Entropy Method (CEM) Configuration
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '8px', textAlign: 'center' }}>
          <div style={{ background: 'rgba(255,255,255,0.03)', padding: '8px', borderRadius: '8px' }}>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>Horizon</div>
            <div style={{ fontSize: '14px', fontWeight: 600, color: '#fff' }}>{model.cemConfig.horizon} steps</div>
          </div>
          <div style={{ background: 'rgba(255,255,255,0.03)', padding: '8px', borderRadius: '8px' }}>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>Samples</div>
            <div style={{ fontSize: '14px', fontWeight: 600, color: '#fff' }}>{model.cemConfig.numSamples}</div>
          </div>
          <div style={{ background: 'rgba(255,255,255,0.03)', padding: '8px', borderRadius: '8px' }}>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>Iterations</div>
            <div style={{ fontSize: '14px', fontWeight: 600, color: '#fff' }}>{model.cemConfig.iterations}</div>
          </div>
          <div style={{ background: 'rgba(255,255,255,0.03)', padding: '8px', borderRadius: '8px' }}>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>Elites</div>
            <div style={{ fontSize: '14px', fontWeight: 600, color: '#fff' }}>{model.cemConfig.numElites}</div>
          </div>
        </div>
      </div>
    </div>
  );
};
