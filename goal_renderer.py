"""Goal-image renderer for the JEPA planner (P4).

Renders a top-down 224x224 BGR view of a DESIRED final block configuration,
pixel-compatible with workspace_env.render_synthetic_camera(): same pinhole
intrinsics (focal_length=200, cx=cy=112), same (50,50,50) background, same
RGB->BGR colour mapping, same ArUco overlay blend (roi 0.3 / marker 0.7 from
pre-rendered DICT_4X4_50 markers at 128 px), same z<0.01 skip rule and same
size clamp max(4, min(size, 224)).

Deviation from the live pipeline (documented): camera_link in workspace_env
hangs off the MOVING manipulator_link with a +0.05 m forward offset; because
the wrist pitch cancels in FK (theta4 = -(theta2 + theta3)), the mounted
camera actually looks horizontally outward and every block sits BEHIND the
camera plane whenever the arm is at rest or above a target (depth < 0.01 ->
skipped by workspace_env itself). A literal replication would therefore
render an empty frame. This module keeps the exact manipulator_link ->
camera_link transform (offset [0.05, 0, -0.02], quat xyzw
[0, 0.7071068, 0, 0.7071068]) and composes it with a FIXED overhead
base_link -> manipulator_link mount so the virtual camera looks straight
down at the table centre from 0.87 m: robot-forward (+x) points image-up,
robot-left (+y) points image-left.

Pure numpy/cv2; transforms are composed manually (no rclpy / tf2 imports).

Used by test_goal_renderer.py: pixi run python3 test_goal_renderer.py
"""
import cv2
import numpy as np

# Block conventions mirrored from stacking_controller.py (importing it would
# pull in rclpy/torch).
COLOR_TO_ID = {'red': 0, 'green': 1, 'blue': 2, 'yellow': 3}
BLOCK_HOME = {0: [0.15, 0.1, 0.02], 1: [0.20, 0.1, 0.02],
              2: [0.15, -0.1, 0.02], 3: [0.20, -0.1, 0.02]}
# RGB unit colours per block id, exactly workspace_env.initial_blocks.
ID_TO_COLOR_RGB = {0: (1.0, 0.0, 0.0), 1: (0.0, 1.0, 0.0),
                   2: (0.0, 0.0, 1.0), 3: (1.0, 1.0, 0.0)}

BLOCK_SIZE = 0.04   # m, cube edge as published in the RViz markers
STACK_STEP = 0.04   # m, z gain per stacking level

# --- Camera model, mirrored line-for-line from render_synthetic_camera() ---
FOCAL_LENGTH = 200.0
CX = CY = 112.0
IMG_SIZE = 224
BG_GRAY = 50

# manipulator_link -> camera_link, exactly as broadcast in
# workspace_env.publish_markers_and_tf().
_CAM_T_MANIP = np.array([0.05, 0.0, -0.02])
_CAM_Q_MANIP_XYZW = (0.0, 0.7071068, 0.0, 0.7071068)

# Fixed base_link -> manipulator_link mount for the virtual goal camera.
# Chosen so that, after composing the exact pitch-90 deg camera transform
# above, the optics look straight down at the table centre (0.175, 0) from
# 0.87 m; quat (-0.5, 0.5, 0.5, 0.5) is a 120 deg twist about -(1,1,1).
# Height picked so stacked levels land in the same integer size bucket
# (int(FOCAL_LENGTH * 0.04 / depth) floors to 9 px for both z=0.02 and
# z=0.06 blocks).
_MOUNT_T_BASE = np.array([0.175, 0.02, 0.92])
_MOUNT_Q_BASE_XYZW = (-0.5, 0.5, 0.5, 0.5)

# Pre-rendered DICT_4X4_50 markers at 128 px (recreated locally so this
# module stays ROS-free; generating at tiny/odd sizes is unreliable, which is
# why the cache uses a fixed high resolution like workspace_env does).
_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ARUCO_CACHE = {
    i: cv2.cvtColor(cv2.aruco.generateImageMarker(_aruco_dict, i, 128),
                    cv2.COLOR_GRAY2BGR)
    for i in range(4)
}


def _quat_xyzw_to_matrix(xyzw):
    x, y, z, w = xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _compose(R_parent_child, t_parent_child):
    """4x4 homogeneous parent<-child transform from R, t."""
    T = np.eye(4)
    T[:3, :3] = R_parent_child
    T[:3, 3] = t_parent_child
    return T


def camera_pose_in_base():
    """4x4 base_link<-camera_link transform.

    Replicates lookup_transform('base_link', 'camera_link') for the fixed
    mount: base<-manip composed with manip<-camera.
    """
    R_mount = _quat_xyzw_to_matrix(_MOUNT_Q_BASE_XYZW)
    R_cam = _quat_xyzw_to_matrix(_CAM_Q_MANIP_XYZW)
    T_base_manip = _compose(R_mount, _MOUNT_T_BASE)
    T_manip_cam = _compose(R_cam, _CAM_T_MANIP)
    return T_base_manip @ T_manip_cam


def project_block(pos_in_base, T_base_cam=None):
    """Project one block centre into the virtual goal camera.

    Returns (u, v, depth, size) using EXACTLY the arithmetic of
    render_synthetic_camera(): u/v are int()-truncated pixel coords, depth is
    the camera-frame z (skip when < 0.01), size is the clamped square edge.
    """
    if T_base_cam is None:
        T_base_cam = camera_pose_in_base()
    R = T_base_cam[:3, :3]
    t = T_base_cam[:3, 3]
    p_cam = R.T @ (np.asarray(pos_in_base, dtype=float) - t)
    x, y, z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])

    u = int(CX + FOCAL_LENGTH * (x / z))
    v = int(CY + FOCAL_LENGTH * (y / z))

    # Camera looks along +z of camera_link; behind/too-close blocks are
    # reported with a degenerate size so callers can skip them like the env.
    if z < 0.01:
        return u, v, z, 0

    size = int(FOCAL_LENGTH * (BLOCK_SIZE / z))
    size = max(4, min(size, IMG_SIZE))  # clamp: avoid degenerate sizes near camera
    return u, v, z, size


def home_blocks():
    """Block list at the BLOCK_HOME layout (id + pos dicts)."""
    return [{'id': bid, 'pos': list(BLOCK_HOME[bid])} for bid in sorted(BLOCK_HOME)]


def stacked_config(base_block_id, moving_ids, stack_xy=None):
    """Describe a desired final stack as a render_goal() block list.

    Base block sits at stack_xy (or its BLOCK_HOME position when None);
    each moving id is stacked directly on top of the previous level, gaining
    STACK_STEP in z while sharing the stack xy, in the given order.
    """
    if stack_xy is not None:
        base_pos = [float(stack_xy[0]), float(stack_xy[1]), float(BLOCK_HOME[base_block_id][2])]
    else:
        base_pos = list(BLOCK_HOME[base_block_id])
    blocks = [{'id': int(base_block_id), 'pos': base_pos}]
    z = base_pos[2]
    for mid in moving_ids:
        z += STACK_STEP
        blocks.append({'id': int(mid), 'pos': [base_pos[0], base_pos[1], z]})
    return blocks


def render_goal(blocks, use_aruco=True):
    """Render the desired configuration into a 224x224 uint8 BGR image."""
    img = np.ones((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8) * BG_GRAY
    T_base_cam = camera_pose_in_base()

    for block in blocks:
        try:
            bid = int(block['id'])
            u, v, z, size = project_block(block['pos'], T_base_cam)

            # Behind or too close to camera (mirrors the env skip rule)
            if z < 0.01:
                continue

            color_rgb = ID_TO_COLOR_RGB[bid]
            bgr = (int(color_rgb[2] * 255), int(color_rgb[1] * 255),
                   int(color_rgb[0] * 255))

            # Bounds for cropping, identical to the env
            x1 = max(0, u - size // 2)
            y1 = max(0, v - size // 2)
            x2 = min(IMG_SIZE, u + size // 2)
            y2 = min(IMG_SIZE, v + size // 2)

            if x1 < x2 and y1 < y2:
                cv2.rectangle(img, (x1, y1), (x2, y2), bgr, -1)

                if use_aruco:
                    marker_img = cv2.resize(_ARUCO_CACHE[bid], (size, size))

                    mx1 = x1 - (u - size // 2)
                    my1 = y1 - (v - size // 2)
                    mx2 = mx1 + (x2 - x1)
                    my2 = my1 + (y2 - y1)

                    marker_crop = marker_img[my1:my2, mx1:mx2]
                    roi = img[y1:y2, x1:x2]

                    if roi.shape == marker_crop.shape and roi.size > 0:
                        blended = cv2.addWeighted(roi, 0.3, marker_crop, 0.7, 0)
                        img[y1:y2, x1:x2] = blended
        except Exception:
            # One bad entry must not blank the whole goal (mirrors the env's
            # per-block try/except behaviour)
            continue
    return img
