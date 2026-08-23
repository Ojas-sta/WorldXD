import React, { useEffect, useRef } from 'react';
import { Eye } from 'lucide-react';
import { RobotState } from '../types';

interface Props {
  robot: RobotState;
}

export const Visualizer: React.FC<Props> = React.memo(({ robot }) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let animFrameId: number;

    const render = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      const w = canvas.width;
      const h = canvas.height;

      // Background
      ctx.fillStyle = '#0a0c12';
      ctx.fillRect(0, 0, w, h);

      // Simple grid lines (fast rendering, no blur)
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
      ctx.lineWidth = 1;
      const gridSize = 25;
      for (let x = 0; x < w; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
      }
      for (let y = 0; y < h; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }

      const mapX = (x: number) => 40 + x * 500;
      const mapY = (y: number) => h / 2 - y * 500;

      // Robot Base
      const basePxX = mapX(0);
      const basePxY = mapY(0);
      ctx.fillStyle = '#2d3345';
      ctx.beginPath();
      ctx.arc(basePxX, basePxY, 16, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#0A84FF';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Trajectory line to EE
      const eePxX = mapX(robot.eePos[0]);
      const eePxY = mapY(robot.eePos[1]);

      ctx.beginPath();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = 'rgba(100, 210, 255, 0.6)';
      ctx.lineWidth = 2;
      ctx.moveTo(basePxX, basePxY);
      ctx.lineTo(eePxX, eePxY);
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw Target Blocks
      const blockColors: Record<string, string> = {
        Red: '#FF453A',
        Green: '#30D158',
        Blue: '#0A84FF',
        Yellow: '#FFD60A'
      };

      robot.blocks.forEach((b) => {
        const bx = mapX(b.pos[0]);
        const by = mapY(b.pos[1]);
        ctx.fillStyle = blockColors[b.color] || '#FFF';
        ctx.fillRect(bx - 10, by - 10, 20, 20);
        ctx.strokeStyle = 'rgba(255,255,255,0.8)';
        ctx.lineWidth = 1;
        ctx.strokeRect(bx - 10, by - 10, 20, 20);
      });

      // End-Effector Dot
      ctx.fillStyle = '#64D2FF';
      ctx.beginPath();
      ctx.arc(eePxX, eePxY, 7, 0, Math.PI * 2);
      ctx.fill();
    };

    animFrameId = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animFrameId);
  }, [robot]);

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '14px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Eye size={18} color="var(--accent-teal)" />
          <h2 style={{ fontSize: '15px', fontWeight: 600 }}>Top-Down Visual Workspace</h2>
        </div>
        <span className="badge badge-green">Live Telemetry</span>
      </div>

      <div style={{ position: 'relative', width: '100%', borderRadius: '12px', overflow: 'hidden', border: '1px solid rgba(255, 255, 255, 0.08)' }}>
        <canvas ref={canvasRef} width={460} height={210} style={{ width: '100%', height: '210px', display: 'block' }} />
        <div style={{ position: 'absolute', bottom: '8px', left: '10px', fontSize: '10px', color: 'rgba(255,255,255,0.5)', background: 'rgba(0,0,0,0.6)', padding: '3px 7px', borderRadius: '5px' }}>
          Base [0, 0] • Table 0.40m × 0.40m
        </div>
      </div>

      <div style={{ display: 'flex', gap: '6px', overflowX: 'auto' }}>
        {robot.blocks.map((b) => (
          <div key={b.id} style={{ background: 'rgba(255,255,255,0.03)', padding: '6px 10px', borderRadius: '6px', border: '1px solid rgba(255,255,255,0.05)', flex: 1 }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '5px', fontSize: '11px', fontWeight: 600 }}>
              <span style={{ width: '7px', height: '7px', borderRadius: '50%', background: b.color.toLowerCase() }} />
              {b.color}
            </div>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)' }}>
              [{b.pos[0]}, {b.pos[1]}]
            </div>
          </div>
        ))}
      </div>
    </div>
  );
});
