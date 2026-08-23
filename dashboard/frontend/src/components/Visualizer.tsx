import React, { useEffect, useRef } from 'react';
import { Eye, MapPin } from 'lucide-react';
import { RobotState } from '../types';

interface Props {
  robot: RobotState;
}

export const Visualizer: React.FC<Props> = ({ robot }) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Grid background
    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = '#0f121a';
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
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

    // Mapping spatial (x: 0..0.4m, y: -0.2..0.2m) to Canvas pixels
    const mapX = (x: number) => 40 + x * 500;
    const mapY = (y: number) => h / 2 - y * 500;

    // Draw robot base at (0, 0)
    const basePxX = mapX(0);
    const basePxY = mapY(0);
    ctx.fillStyle = '#3a3f52';
    ctx.beginPath();
    ctx.arc(basePxX, basePxY, 18, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = 'var(--accent-blue)';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Draw Robot End-Effector Trajectory path to target
    const eePxX = mapX(robot.eePos[0]);
    const eePxY = mapY(robot.eePos[1]);

    ctx.beginPath();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(100, 210, 255, 0.5)';
    ctx.lineWidth = 2;
    ctx.moveTo(basePxX, basePxY);
    ctx.lineTo(eePxX, eePxY);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw Blocks
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
      ctx.shadowColor = blockColors[b.color] || '#FFF';
      ctx.shadowBlur = 8;
      ctx.fillRect(bx - 12, by - 12, 24, 24);
      ctx.shadowBlur = 0;

      ctx.strokeStyle = '#FFFFFF';
      ctx.lineWidth = 1;
      ctx.strokeRect(bx - 12, by - 12, 24, 24);
    });

    // Draw End Effector Position Dot
    ctx.fillStyle = '#FFFFFF';
    ctx.shadowColor = 'var(--accent-teal)';
    ctx.shadowBlur = 12;
    ctx.beginPath();
    ctx.arc(eePxX, eePxY, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;
  }, [robot]);

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '14px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Eye size={20} color="var(--accent-teal)" />
          <h2 style={{ fontSize: '16px', fontWeight: 600 }}>Top-Down Visual Workspace</h2>
        </div>
        <span className="badge badge-green">30 FPS Live Stream</span>
      </div>

      <div style={{ position: 'relative', width: '100%', borderRadius: '14px', overflow: 'hidden', border: '1px solid rgba(255, 255, 255, 0.1)' }}>
        <canvas ref={canvasRef} width={460} height={220} style={{ width: '100%', height: '220px', display: 'block' }} />
        <div style={{ position: 'absolute', bottom: '10px', left: '12px', fontSize: '11px', color: 'rgba(255,255,255,0.6)', background: 'rgba(0,0,0,0.5)', padding: '4px 8px', borderRadius: '6px', backdropFilter: 'blur(8px)' }}>
          Workspace Coordinates • Base [0, 0] • Table 0.40m × 0.40m
        </div>
      </div>

      <div style={{ display: 'flex', gap: '8px', overflowX: 'auto', paddingBottom: '4px' }}>
        {robot.blocks.map((b) => (
          <div key={b.id} style={{ background: 'rgba(255,255,255,0.04)', padding: '8px 12px', borderRadius: '8px', border: '1px solid rgba(255,255,255,0.06)', minWidth: '100px', flex: 1 }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '11px', fontWeight: 600 }}>
              <span style={{ width: '8px', height: '8px', borderRadius: '50%', background: b.color.toLowerCase() }} />
              {b.color} Box
            </div>
            <div style={{ fontSize: '10px', color: 'var(--text-tertiary)', marginTop: '2px' }}>
              [{b.pos[0]}, {b.pos[1]}]
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};
