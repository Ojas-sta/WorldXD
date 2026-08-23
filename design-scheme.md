# Apple Design Scheme — WorldXD UI Reference

> **Rule:** every UI change in this repo (web dashboard, TUI, any future GUI) must follow
> this document. Saved verbatim from the apple-design skill; see Onboarding.md §4.

---

How Apple builds interfaces that stop feeling like a computer and start feeling like an extension of you. This knowledge comes from Apple's WWDC design talks — chiefly *Designing Fluid Interfaces* (WWDC 2018) — distilled and translated into the web platform (CSS, Pointer Events, `requestAnimationFrame`, spring libraries like Motion/Framer Motion).

The through-line: **an interface feels alive when motion starts from the current on-screen value, inherits the user's velocity, projects momentum forward, and can be grabbed and reversed at any instant.** Springs are the tool that makes all of this natural, because they are inherently interruptible and velocity-aware.

## The Core Idea

> "When we align the interface to the way we think and move, something magical happens — it stops feeling like a computer and starts feeling like an extension of us."

An interface is fluid when it behaves like the physical world: things respond instantly, move continuously, carry momentum, resist at boundaries, and can be redirected mid-motion. Everything below is a way to get closer to that.

Apple frames design as serving four human needs: **safety/predictability, understanding, achievement, and joy.** Every rule here serves one of them.

## 1. Response — kill latency

The moment lag appears, the feeling of directness "falls off a cliff." Response is the foundation everything else is built on.

- **Respond on pointer-down, not on release.** Highlight a button the instant it's pressed. Waiting for `click`/touch-up to show feedback feels dead.
- **Be vigilant about every latency.** Audit debounces, artificial timers, transition waits, and the ~300ms tap delay. Anything on the input path that isn't essential is a regression.
- **Feedback must be continuous *during* the interaction, not just at the end.** For a drag, slider, or drawer, update the UI 1:1 with the pointer the whole way through — never animate only when the gesture completes.

```css
/* Feedback lives on the press, and it's instant */
.button:active {
  transform: scale(0.97);
  transition: transform 100ms ease-out;
}
```

## 2. Direct manipulation — 1:1 tracking

> "Touch and content should move together."

When the user drags something, it must stay glued to the finger — and respect the offset from *where they grabbed it*. Snapping to the element's center on grab breaks the illusion immediately.

- Use Pointer Events with `setPointerCapture` so tracking continues even when the pointer leaves the element's bounds.
- Track a short **velocity/position history** (last few `pointermove` events), not just the current point — you'll need velocity at release.

```js
el.addEventListener('pointerdown', (e) => {
  el.setPointerCapture(e.pointerId);
  const grabOffset = e.clientY - el.getBoundingClientRect().top; // respect where they grabbed
  // ...track position + timestamp history for velocity
});
```

## 3. Interruptibility — the single most important principle

> "The thought and the gesture happen in parallel."

Every animation must be interruptible and redirectable at any moment. A user must be able to grab a moving element mid-flight and reverse it without waiting for the animation to finish.

- **Never lock out input during a transition.**
- **Always animate from the *presentation* (current) value, never the target value.**
- **Avoid CSS transitions and `@keyframes` for anything gesture-driven** — they can't be smoothly grabbed and reversed mid-flight. Springs animate from the current value by default.
- **When a gesture reverses, blend velocity — don't hard-cut it.**
- **Decompose 2D motion into independent X and Y springs.**

## 4. Behavior over animation — use springs

A pre-scripted, fixed-duration animation can't respond to new input. A spring can.

Apple deliberately replaced the physics triplet (mass/stiffness/damping) with two designer-friendly parameters:

- **Damping ratio** — controls overshoot. `1.0` = critically damped, no bounce. `< 1.0` = overshoots.
- **Response** — how quickly the value reaches the target, in seconds. **This is not "duration."**

**Defaults:** start most UI at **damping `1.0`**. Add bounce (**~`0.8`**) only when the gesture itself carried momentum.

| Interaction | Damping | Response |
| --- | --- | --- |
| Move / reposition | `1.0` | `0.4` |
| Rotation | `0.8` | `0.4` |
| Drawer / sheet | `0.8` | `0.3` |

```js
import { animate } from 'motion';

// Critically damped default (no overshoot)
animate(el, { y: 0 }, { type: 'spring', bounce: 0, duration: 0.4 });

// Momentum interaction — a little bounce, only because a flick preceded it
animate(el, { y: target }, { type: 'spring', bounce: 0.2, duration: 0.4 });
```

## 5. Velocity handoff — the seam between drag and animation

When a gesture ends, the animation must **continue at the finger's exact velocity**:

```
relativeVelocity = gestureVelocity / (targetValue − currentValue)
```

Framer Motion / Motion take absolute px/s velocity directly (`velocity` option).

## 6. Momentum projection — animate to where the gesture is *going*

Use velocity to **project the resting position**, then snap to the nearest target from the projection:

```js
function project(initialVelocity /* px/s */, decelerationRate = 0.998) {
  return (initialVelocity / 1000) * decelerationRate / (1 - decelerationRate);
}

const projectedEndpoint = currentPosition + project(releaseVelocity);
const target = nearestSnapPoint(projectedEndpoint);
animateSpringTo(target, { velocity: releaseVelocity });
```

## 7. Spatial consistency — symmetric paths, anchored origins

- **Enter and exit along the same path.**
- **Anchor interactions to their source** (`transform-origin` = trigger).
- **Mirror the easing on reversible transitions.**

## 8. Hint in the direction of the gesture

Intermediate motion should telegraph where things are going.

## 9. Rubber-banding — soft boundaries

At an edge, resist progressively instead of stopping hard:

```js
function rubberband(overshoot, dimension, constant = 0.55) {
  return (overshoot * dimension * constant) / (dimension + constant * Math.abs(overshoot));
}
```

## 10. Gesture design details (the "feel" checklist)

- **Tap:** highlight on touch-*down*, commit on touch-*up*. ~10px hysteresis; cancel-by-dragging-away.
- **Drag/swipe:** small movement threshold (~10px) before committing, then track 1:1.
- **Detect all plausible gestures in parallel**, cancel losers once intent is clear.

## 11. Frame-level smoothness

- Animate only compositor-friendly properties — `transform` and `opacity`.
- `requestAnimationFrame` is the display-synced clock.
- For fast motion, subtle motion blur/stretch encodes speed.

## 12. Materials & depth — translucency conveys hierarchy

- **Build nav/toolbars/sheets as translucent layers** (`backdrop-filter: blur()` + semi-transparent background) with content scrolling underneath.
- **Material weight encodes hierarchy. Never stack a light translucent surface on another.**
- **Bigger surfaces read as thicker:** stronger blur + deeper shadow.
- **Dim to focus** (modal = scrim), **translucent offset for non-blocking panels** (no scrim).
- **Vibrancy keeps text legible over translucent surfaces** — higher contrast, slightly heavier weight, letter-spacing bump; color on solid layers only.
- **Scroll edge effects, not hard dividers.**
- **Materialize, don't just fade** — animate blur radius and scale together.

```css
.toolbar {
  background: rgba(255, 255, 255, 0.6);
  backdrop-filter: blur(20px) saturate(180%);
  border-top: 1px solid rgba(255, 255, 255, 0.4); /* bright top edge */
}
```

## 13. Multimodal feedback — motion + sound + haptics

1. **Causality** — trigger on the causal event, match physicality.
2. **Harmony** — visual/sound/haptic fire on the same frame.
3. **Utility** — feedback only where it earns its place.

## 14. Reduced motion & accessibility

- **`prefers-reduced-motion: reduce`** — cross-fades instead of slides/springs; drop overshoot.
- **`prefers-reduced-transparency: reduce`** — frostier surfaces, raise opacity, drop blur.
- **`prefers-contrast: more`** — near-solid backgrounds with defined borders.
- Avoid full-viewport moving backgrounds, slow oscillations (~0.2Hz), abrupt brightness jumps.

```css
@media (prefers-reduced-motion: reduce) {
  .sheet { transition: opacity 200ms ease; transform: none !important; }
}
@media (prefers-reduced-transparency: reduce) {
  .toolbar { background: white; backdrop-filter: none; }
}
```

## 15. Typography — optical sizing, tracking, leading

- **Tracking is size-specific — never one value for all sizes.** Tighten large text (`-0.02em`), body near `0`.
- **Leading tracks size inversely** — tight headings, looser body.
- **Hierarchy from weight + size + leading as a set.**
- **Respect Dynamic Type** — spacing in rem/em.
- **Default to system font.**

```css
:root { font: 100%/1.5 system-ui, sans-serif; }

.display {
  font-size: clamp(2rem, 5vw, 4rem);
  line-height: 1.05;
  letter-spacing: -0.02em;
  font-optical-sizing: auto;
}
```

## 16. Design foundations — the eight principles

1. **Purpose.** Decide what *not* to build; spend attention only where it pays off.
2. **Agency.** People in control; forgiveness (undo); confirmation only for truly destructive actions.
3. **Responsibility.** Privacy at the right moment; anticipate harm; previews and warnings.
4. **Familiarity.** Known metaphors; consistent placement/behavior; break patterns only with proof.
5. **Flexibility.** Adapt to platform, context, abilities; allow personalization.
6. **Simplicity — not minimalism.** Strip the unnecessary; concise language; hierarchy; common path first.
7. **Craft.** Every spacing/timing/alignment defensible; nothing random; iteration and longevity.
8. **Delight.** The result of the other seven, not confetti on top.

Tactical rules:
- Feedback in four kinds: status, completion, warning, error. Validate inline.
- **Wayfinding:** every screen answers where am I / where can I go / what's there / how do I get out.
- **Grouping & mapping:** control near what it affects.
- **Specific labels beat generic ones** ("Progress", "Library" — not "Home").

## 17. Process

- **Prototype interactively** — a working demo beats static designs and sets the bar.
- **Design interaction and visuals together.**
- **Test with real people in real context; review motion frame-by-frame.**

## Quick Reference

| Need | Technique | Concrete value |
| --- | --- | --- |
| Default UI spring | Critically damped | `damping 1.0`, `response 0.3–0.4` |
| Momentum / flick spring | Under-damped slightly | `damping ~0.8`, `response 0.3–0.4` |
| Gesture → spring velocity | Hand off release velocity | `gestureVelocity / (target − current)` if normalized |
| Flick landing point | Project momentum | `current + (v/1000)·d/(1−d)`, `d ≈ 0.998` |
| Interrupt cleanly | Start from live value | read on-screen transform |
| Avoid reversal brick wall | Carry velocity through re-target | spring blends velocity |
| Reversible transition | Mirror easing curve | inverse cubic-bézier |
| Reverse vs commit | Velocity **sign**, not position | at release |
| 1:1 drag | Pointer Events + capture | respect grab offset |
| Feedback | On pointer-down, continuous | never only at end |
| Boundary | Rubber-band | progressive resistance |
| Translucent chrome | `backdrop-filter` layer | content scrolls under |
| Type tracking | Size-specific | `-0.02em` large, `0` body |
| Reduced motion | Cross-fade | `@media (prefers-reduced-motion)` |
