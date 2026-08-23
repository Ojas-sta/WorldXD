import React, { useEffect, useState } from 'react';
import { Camera, Wifi, WifiOff } from 'lucide-react';
import { io } from 'socket.io-client';

interface Props {
  socket: ReturnType<typeof io> | null;
  resolution: [number, number] | null;
}

// Live feed of the synthetic camera (/camera/image_raw via ros_bridge -> server).
// Frames arrive as base64 JPEG on the 'camera' socket event (~8 fps).
export const CameraFeed: React.FC<Props> = ({ socket, resolution }) => {
  const [frame, setFrame] = useState<string | null>(null);
  const [lastFrameTime, setLastFrameTime] = useState<number>(0);
  const [stale, setStale] = useState(true);

  useEffect(() => {
    if (!socket) return;
    const onCamera = (jpeg: string) => {
      setFrame(`data:image/jpeg;base64,${jpeg}`);
      setLastFrameTime(Date.now());
      setStale(false);
    };
    socket.on('camera', onCamera);
    return () => {
      socket.off('camera', onCamera);
    };
  }, [socket]);

  // Mark feed stale if no frame for >2s (node down / sim stopped)
  useEffect(() => {
    const t = setInterval(() => {
      setStale(Date.now() - lastFrameTime > 2000);
    }, 1000);
    return () => clearInterval(t);
  }, [lastFrameTime]);

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <Camera size={18} color="var(--accent-purple)" />
          <h2 style={{ fontSize: '15px', fontWeight: 600 }}>Onboard Camera</h2>
        </div>
        <span className="badge" style={{
          background: stale ? 'rgba(255,69,58,0.15)' : 'rgba(48,209,88,0.15)',
          color: stale ? '#FF453A' : '#30D158',
          border: `1px solid ${stale ? 'rgba(255,69,58,0.25)' : 'rgba(48,209,88,0.25)'}`,
          display: 'inline-flex', alignItems: 'center', gap: '5px'
        }}>
          {stale ? <WifiOff size={11} /> : <Wifi size={11} />}
          {stale ? 'No Signal' : 'Live'}
        </span>
      </div>

      <div style={{
        position: 'relative', width: '100%', aspectRatio: '1 / 1',
        borderRadius: '12px', overflow: 'hidden',
        border: '1px solid rgba(255,255,255,0.08)', background: '#0a0c12'
      }}>
        {frame && !stale ? (
          <img src={frame} alt="camera feed"
            style={{ width: '100%', height: '100%', objectFit: 'cover', display: 'block' }} />
        ) : (
          <div style={{
            width: '100%', height: '100%', display: 'flex',
            alignItems: 'center', justifyContent: 'center',
            color: 'rgba(255,255,255,0.3)', fontSize: '12px'
          }}>
            Waiting for /camera/image_raw ...
          </div>
        )}
        <div style={{
          position: 'absolute', bottom: '8px', left: '10px',
          fontSize: '10px', color: 'rgba(255,255,255,0.5)',
          background: 'rgba(0,0,0,0.6)', padding: '3px 7px', borderRadius: '5px'
        }}>
          {resolution ? `${resolution[0]}×${resolution[1]}` : '—'} · synthetic · ~8 fps
        </div>
      </div>
    </div>
  );
};
