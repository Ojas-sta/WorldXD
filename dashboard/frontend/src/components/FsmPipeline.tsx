import React from 'react';
import { motion, useReducedMotion } from 'framer-motion';
import { GitBranch } from 'lucide-react';

interface Props {
  current: string;
  order: string[];
}

const LABELS: Record<string, string> = {
  MANUAL: 'Manual Jog',
  MOVE_ABOVE_BLOCK: 'Approach',
  DESCEND: 'Descend',
  CLOSE_GRIPPER: 'Grip',
  LIFT: 'Lift',
  MOVE_ABOVE_STACK: 'Transport',
  PLACE: 'Place',
  OPEN_GRIPPER: 'Release',
  RETREAT: 'Retreat',
  DONE: 'Idle'
};

// Live view of the controller state machine (/fsm_state). Active chip is
// highlighted; transitions use a critically damped spring per design-scheme.md §4.
export const FsmPipeline: React.FC<Props> = ({ current, order }) => {
  const reduceMotion = useReducedMotion();
  const activeIdx = order.indexOf(current);
  const busy = activeIdx !== -1;

  return (
    <div className="apple-glass-card" style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <GitBranch size={18} color="var(--accent-green)" />
          <h2 style={{ fontSize: '15px', fontWeight: 600 }}>State Machine</h2>
        </div>
        <span className="badge" style={{
          background: busy ? 'rgba(48,209,88,0.15)' : 'rgba(255,255,255,0.06)',
          color: busy ? '#30D158' : 'var(--text-tertiary)',
          border: `1px solid ${busy ? 'rgba(48,209,88,0.25)' : 'rgba(255,255,255,0.1)'}`
        }}>
          {LABELS[current] || current}
        </span>
      </div>

      <div style={{ display: 'flex', alignItems: 'center', gap: '4px', flexWrap: 'wrap' }}>
        {order.map((state, idx) => {
          const isActive = state === current;
          // MANUAL is a mode, not a sequence step: never render it as "done"
          const isDone = busy && activeIdx > idx && state !== 'MANUAL';
          return (
            <React.Fragment key={state}>
              {idx > 0 && (
                <span style={{
                  color: isDone ? 'rgba(48,209,88,0.6)' : 'rgba(255,255,255,0.15)',
                  fontSize: '11px', flexShrink: 0
                }}>→</span>
              )}
              <motion.div
                animate={isActive ? { scale: 1.06 } : { scale: 1 }}
                transition={reduceMotion
                  ? { duration: 0 }
                  : { type: 'spring', damping: 1.0, duration: 0.3 }}
                style={{
                  padding: '5px 9px',
                  borderRadius: '7px',
                  fontSize: '11px',
                  fontWeight: isActive ? 600 : 400,
                  whiteSpace: 'nowrap',
                  background: isActive
                    ? 'rgba(10, 132, 255, 0.22)'
                    : isDone
                      ? 'rgba(48, 209, 88, 0.12)'
                      : 'rgba(255,255,255,0.04)',
                  color: isActive
                    ? '#64D2FF'
                    : isDone
                      ? 'rgba(48,209,88,0.8)'
                      : 'var(--text-tertiary)',
                  border: `1px solid ${isActive ? 'rgba(100,210,255,0.4)' : 'transparent'}`
                }}
              >
                {LABELS[state]}
              </motion.div>
            </React.Fragment>
          );
        })}
      </div>
    </div>
  );
};
