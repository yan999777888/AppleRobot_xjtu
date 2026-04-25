#!/usr/bin/env python3
"""
SR4 + ZED 2 Eye-to-Hand

    camera fixed in workspace, checkerboard on end-effector
    cv2.calibrateHandEye  AX=XB

1.  ZED 2
2.  BASE_POSE
3.
4.
5.  calibration_results_eye_to_hand/
"""

import sys
import os

curPath = os.path.abspath(os.path.dirname(__file__))
rootPath = os.path.split(curPath)[0] + '/lib'
sys.path.append(rootPath)

import numpy as np
import time
import cv2
import pyzed.sl as sl
from scipy.spatial.transform import Rotation
import json
from datetime import datetime
from robot import *


# ================================================================
#
# ================================================================

ROBOT_IP = "192.168.2.160"

# ---  ---
CHECKERBOARD_ROWS = 5          #
CHECKERBOARD_COLS = 5          #
SQUARE_SIZE = 0.0285            #

# ---  ---
MOVE_SPEED = 200               #
SETTLE_TIME = 2.0              #
NUM_CAPTURES = 5               #

# ---  ---
# :  flangePos
# flangePos
# None =
BASE_POSE = None

# ()
POS_DELTA_X = 0.06
POS_DELTA_Y = 0.06
POS_DELTA_Z = 0.04

# ()  ~15
ROT_DELTA = 0.25

#
NUM_POSES = 20

# cv2.CALIB_HAND_EYE_TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS
HAND_EYE_METHOD = cv2.CALIB_HAND_EYE_TSAI

# ZED 2
ZED_RESOLUTION = sl.RESOLUTION.HD720
ZED_DEPTH_MODE = sl.DEPTH_MODE.ULTRA
ZED_FPS = 15

#  ()
WAIT_TIMEOUT = 30.0

#  ()
MIN_POS_DIST = 0.02    # 20mm
MIN_ROT_DIST = 5.0     # deg

#
SAVE_DIR = os.path.join(curPath, "calibration_results_eye_to_hand")


# ================================================================
#
# ================================================================

def euler_to_rotation_matrix(rx, ry, rz):
    return Rotation.from_euler('xyz', [rx, ry, rz]).as_matrix()


def pose_to_matrix(pose):
    """[x, y, z, rx, ry, rz] -> 4x4"""
    T = np.eye(4)
    T[:3, :3] = euler_to_rotation_matrix(pose[3], pose[4], pose[5])
    T[:3, 3] = [pose[0], pose[1], pose[2]]
    return T


def matrix_to_pose(T):
    """4x4 -> [x, y, z, rx, ry, rz]"""
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
    return [T[0, 3], T[1, 3], T[2, 3], rpy[0], rpy[1], rpy[2]]


def show_camera(camera, text="", board_size=None, square_size=None):
    """capture one frame, overlay text, show in window. Returns the color image."""
    color_img, _ = camera.capture()
    if color_img is None:
        return None
    display = color_img.copy()
    if board_size is not None and square_size is not None:
        found, _, _, vis = detect_checkerboard(
            color_img, camera.camera_matrix, camera.dist_coeffs,
            board_size, square_size
        )
        if found:
            display = vis
    if text:
        cv2.putText(display, text, (10, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Eye-to-Hand Calibration", display)
    cv2.waitKey(1)
    return color_img


def waitRobot(robot, camera=None, status_text=""):
    """
    Wait for robot to finish, returns:
      True  - motion completed
      False - collision/timeout detected
      None  - user pressed ESC
    """
    start_time = time.time()
    while True:
        time.sleep(0.1)

        if camera is not None:
            show_camera(camera, status_text)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("  ESC")
                return None

        ec = {}
        try:
            st = robot.operationState(ec)
        except Exception:
            return False

        if st == rokae.OperationState.idle.value or st == rokae.OperationState.unknown.value:
            return True

        if time.time() - start_time > WAIT_TIMEOUT:
            print(f"  {WAIT_TIMEOUT}s !")
            return False


def recover_robot(robot):
    ec = {}
    try:
        robot.stop(ec)
    except Exception:
        pass
    time.sleep(1)
    try:
        robot.moveReset(ec)
    except Exception:
        pass
    time.sleep(1)
    try:
        robot.setOperateMode(rokae.OperateMode.automatic, ec)
        robot.setPowerState(True, ec)
    except Exception:
        pass
    time.sleep(1)
    print("  ")


def confirm_detection(vis, pose_idx, total_poses, reference=None):
    """
    Show detection result and wait for user confirmation.
    reference: last accepted vis image, shown as inset for orientation comparison.
    Returns: True=accept  False=reject  None=abort(ESC)
    """
    display = vis.copy()
    h, w = display.shape[:2]

    # reference inset (top-right corner)
    if reference is not None:
        rh, rw = h // 4, w // 4
        inset = cv2.resize(reference, (rw, rh))
        display[8:8+rh, w-rw-8:w-8] = inset
        cv2.rectangle(display, (w-rw-8, 8), (w-8, 8+rh), (0, 255, 255), 2)
        cv2.putText(display, "Last accepted", (w-rw-8, 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    bar = display.copy()
    cv2.rectangle(bar, (0, h - 70), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(bar, 0.55, display, 0.45, 0, display)

    cv2.putText(display,
                f"Pose {pose_idx}/{total_poses}  |  Check axis orientation (RGB arrows)",
                (10, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.putText(display,
                "SPACE = Accept    R = Reject (wrong rotation)    ESC = Abort",
                (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    cv2.imshow("Eye-to-Hand Calibration", display)
    print(f"    SPACE=  R=  ESC=")

    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == 32:
            print("    ")
            return True
        elif key in (ord('r'), ord('R')):
            print("    ")
            return False
        elif key == 27:
            print("    ESC")
            return None


def is_pose_too_similar(T_new, R_list, t_list):
    """Return True if T_new is too close to any already-recorded pose."""
    for R_prev, t_prev in zip(R_list, t_list):
        pos_diff = np.linalg.norm(T_new[:3, 3] - t_prev) * 1000  # mm
        R_diff = T_new[:3, :3] @ R_prev.T
        cos_a = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
        rot_diff = np.degrees(np.arccos(cos_a))
        if pos_diff < MIN_POS_DIST * 1000 and rot_diff < MIN_ROT_DIST:
            return True, pos_diff, rot_diff
    return False, 0.0, 0.0


# ================================================================
# ZED 2
# ================================================================

class ZED2Camera:
    def __init__(self, resolution=ZED_RESOLUTION, depth_mode=ZED_DEPTH_MODE, fps=ZED_FPS):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = resolution
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = depth_mode
        self.init_params.camera_fps = fps
        self.zed_color_img = sl.Mat()
        self.camera_depth_img = sl.Mat()
        self.camera_matrix = None
        self.dist_coeffs = None

    def start(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED 2: {err}")

        calib_params = (self.zed.get_camera_information()
                        .camera_configuration
                        .calibration_parameters
                        .left_cam)
        fx = calib_params.fx
        fy = calib_params.fy
        cx = calib_params.cx
        cy = calib_params.cy

        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        # ZED SDK
        self.dist_coeffs = np.zeros(5)

        for _ in range(30):
            self.zed.grab()

        print(f"  ZED 2")
        print(f"  : fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")

    def capture(self):
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None, None

        self.zed.retrieve_image(self.zed_color_img, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.camera_depth_img, sl.MEASURE.DEPTH)

        color_data = self.zed_color_img.get_data()
        bgr_data = cv2.cvtColor(color_data, cv2.COLOR_RGBA2BGR)

        return bgr_data, self.camera_depth_img

    def stop(self):
        self.zed.close()


# ================================================================
#
# ================================================================

def detect_checkerboard(color_image, camera_matrix, dist_coeffs,
                        board_size, square_size):
    """
    solvePnP T_cam_target ( -> )

    Returns: (found, R_target2cam, t_target2cam, vis_image)
    """
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
             cv2.CALIB_CB_NORMALIZE_IMAGE +
             cv2.CALIB_CB_FAST_CHECK)

    found, corners = cv2.findChessboardCorners(gray, board_size, None, flags)
    if not found:
        return False, None, None, color_image

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

    success, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)
    if not success:
        return False, None, None, color_image

    R_target2cam, _ = cv2.Rodrigues(rvec)

    vis = color_image.copy()
    cv2.drawChessboardCorners(vis, board_size, corners_refined, found)
    cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, square_size * 2)

    return True, R_target2cam, tvec.flatten(), vis


# ================================================================
#
# ================================================================

def generate_poses(base_pose, num_poses, dx, dy, dz, drot):
    np.random.seed(42)
    poses = [list(base_pose)]

    for _ in range(num_poses - 1):
        pose = list(base_pose)
        pose[0] += np.random.uniform(-dx, dx)
        pose[1] += np.random.uniform(-dy, dy)
        pose[2] += np.random.uniform(-dz, dz)
        pose[3] += np.random.uniform(-drot, drot)
        pose[4] += np.random.uniform(-drot, drot)
        pose[5] += np.random.uniform(-drot, drot)
        poses.append(pose)

    return poses


# ================================================================
#
# ================================================================

def compute_consistency_error(R_g2b_list, t_g2b_list,
                              R_t2c_list, t_t2c_list, T_cam2base):
    """
    Eye-to-Hand:
    T_gripper_target = inv(T_base_gripper) @ T_cam2base @ T_cam_target
    , T_gripper_target
    """
    gripper_target_positions = []
    for i in range(len(R_g2b_list)):
        T_bg = np.eye(4)
        T_bg[:3, :3] = R_g2b_list[i]
        T_bg[:3, 3] = t_g2b_list[i]

        T_ct = np.eye(4)
        T_ct[:3, :3] = R_t2c_list[i]
        T_ct[:3, 3] = t_t2c_list[i]

        T_gt = np.linalg.inv(T_bg) @ T_cam2base @ T_ct
        gripper_target_positions.append(T_gt[:3, 3])

    gripper_target_positions = np.array(gripper_target_positions)
    mean_pos = np.mean(gripper_target_positions, axis=0)
    errors = [np.linalg.norm(p - mean_pos) for p in gripper_target_positions]
    return errors, mean_pos


# ================================================================
#
# ================================================================

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    board_size = (CHECKERBOARD_COLS, CHECKERBOARD_ROWS)

    print("=" * 60)
    print("SR4 + ZED 2 (Eye-to-Hand)")
    print("=" * 60)
    print("  : , ")

    # ---- 1.  ----
    print("\n[1/5] ZED 2...")
    camera = ZED2Camera()
    camera.start()

    # ---- 2.  ----
    print("\n[2/5] ...")
    ec = {}
    robot = XMateRobot(ROBOT_IP)
    robot.connectToRobot(ec)
    robot.setOperateMode(rokae.OperateMode.automatic, ec)
    robot.setPowerState(True, ec)
    robot.moveReset(ec)
    robot.setDefaultZone(0, ec)
    robot.setDefaultSpeed(MOVE_SPEED, ec)

    current_pose = robot.flangePos(ec)
    print(f"  : {[f'{v:.4f}' for v in current_pose]}")

    # ---- 3.  ----
    base_pose = BASE_POSE if BASE_POSE is not None else current_pose
    print(f"\n  : {[f'{v:.4f}' for v in base_pose]}")

    cv2.namedWindow("Eye-to-Hand Calibration", cv2.WINDOW_NORMAL)

    print("\n  ...")
    color_img, _ = camera.capture()
    if color_img is not None:
        found, _, _, vis = detect_checkerboard(
            color_img, camera.camera_matrix, camera.dist_coeffs,
            board_size, SQUARE_SIZE
        )
        cv2.imwrite(os.path.join(SAVE_DIR, "preview.png"), vis)
        cv2.imshow("Eye-to-Hand Calibration", vis)
        cv2.waitKey(500)
        if found:
            print("  ")
        else:
            print("  !")
            print("  , ")
            resp = input("  ? (y/n): ").strip().lower()
            if resp != 'y':
                robot.setPowerState(False, ec)
                robot.disconnectFromRobot(ec)
                camera.stop()
                cv2.destroyAllWindows()
                return

    poses = generate_poses(base_pose, NUM_POSES,
                           POS_DELTA_X, POS_DELTA_Y, POS_DELTA_Z, ROT_DELTA)
    print(f"   {len(poses)} ")

    # ---- 4.  ----
    print(f"\n[3/5] ...")
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []
    valid_count = 0
    user_abort = False
    collision_count = 0
    reference_vis = None

    for i, pose in enumerate(poses):
        if user_abort:
            break

        print(f"\n  [{i+1}/{len(poses)}] "
              f"x={pose[0]:.3f} y={pose[1]:.3f} z={pose[2]:.3f} "
              f"rx={pose[3]:.3f} ry={pose[4]:.3f} rz={pose[5]:.3f}")

        try:
            p = MoveJCommand(pose, MOVE_SPEED, 0)
            robot.executeCommand([p], ec)
            time.sleep(1)
            robot.moveStart(ec)
            move_result = waitRobot(
                robot, camera,
                f"Moving to pose {i+1}/{len(poses)}..."
            )

            if move_result is None:
                user_abort = True
                break
            elif move_result is False:
                collision_count += 1
                print(f"    ! ({collision_count})")
                recover_robot(robot)
                continue

        except Exception as e:
            print(f"    : {e}")
            collision_count += 1
            recover_robot(robot)
            continue

        # settle time
        settle_start = time.time()
        while time.time() - settle_start < SETTLE_TIME:
            show_camera(camera, f"Settling... pose {i+1}/{len(poses)}")
            time.sleep(0.05)

        actual_pose = robot.flangePos(ec)

        # duplicate pose check
        T_actual = pose_to_matrix(actual_pose)
        too_close, pd, rd = is_pose_too_similar(
            T_actual, R_gripper2base_list, t_gripper2base_list
        )
        if too_close:
            print(f"    ({pd:.1f}mm / {rd:.1f}deg), ")
            continue

        detections = []
        for _ in range(NUM_CAPTURES):
            color_img, _ = camera.capture()
            if color_img is None:
                continue
            found, R_t2c, t_t2c, vis = detect_checkerboard(
                color_img, camera.camera_matrix, camera.dist_coeffs,
                board_size, SQUARE_SIZE
            )
            if found:
                detections.append((R_t2c, t_t2c, vis))
                cv2.imshow("Eye-to-Hand Calibration", vis)
            else:
                cv2.imshow("Eye-to-Hand Calibration", color_img)
            cv2.waitKey(1)
            time.sleep(0.1)

        if not detections:
            print(f"    ")
            continue

        mid = len(detections) // 2
        R_t2c, t_t2c, vis = detections[mid]

        # --- manual rotation check ---
        confirm = confirm_detection(vis, i + 1, len(poses), reference=reference_vis)
        if confirm is None:
            user_abort = True
            break
        elif confirm is False:
            print(f"    , ")
            continue
        # -----------------------------

        cv2.imwrite(os.path.join(SAVE_DIR, f"pose_{i+1:03d}.png"), vis)

        T_bg = pose_to_matrix(actual_pose)
        R_gripper2base_list.append(T_bg[:3, :3])
        t_gripper2base_list.append(T_bg[:3, 3])
        R_target2cam_list.append(R_t2c)
        t_target2cam_list.append(t_t2c)

        reference_vis = vis
        valid_count += 1
        print(f"    ( {valid_count} )")

    if collision_count > 0:
        print(f"\n   {collision_count} , ")

    #
    print("\n  ...")
    try:
        p_home = MoveJCommand(list(base_pose), MOVE_SPEED, 0)
        robot.executeCommand([p_home], ec)
        time.sleep(1)
        robot.moveStart(ec)
        waitRobot(robot, camera, "Returning home...")
    except Exception:
        pass

    robot.setPowerState(False, ec)
    robot.disconnectFromRobot(ec)
    camera.stop()
    cv2.destroyAllWindows()

    # ---- 5.  ----
    print("\n" + "=" * 60)
    print(f"[4/5] ...  {valid_count} ")

    if valid_count < 3:
        print("  3 !")
        print("  BASE_POSE POS_DELTA / ROT_DELTA")
        return

    np.save(os.path.join(SAVE_DIR, "R_gripper2base.npy"), np.array(R_gripper2base_list))
    np.save(os.path.join(SAVE_DIR, "t_gripper2base.npy"), np.array(t_gripper2base_list))
    np.save(os.path.join(SAVE_DIR, "R_target2cam.npy"), np.array(R_target2cam_list))
    np.save(os.path.join(SAVE_DIR, "t_target2cam.npy"), np.array(t_target2cam_list))
    np.savetxt(os.path.join(SAVE_DIR, "camera_matrix.txt"), camera.camera_matrix)

    # Eye-to-Hand: calibrateHandEye
    # R_base2gripper, t_base2gripper ( flangePos )
    R_base2gripper_list = []
    t_base2gripper_list = []
    for i in range(len(R_gripper2base_list)):
        T_bg = np.eye(4)
        T_bg[:3, :3] = R_gripper2base_list[i]
        T_bg[:3, 3] = t_gripper2base_list[i]
        T_gb = np.linalg.inv(T_bg)
        R_base2gripper_list.append(T_gb[:3, :3])
        t_base2gripper_list.append(T_gb[:3, 3])

    methods = {
        "TSAI":       cv2.CALIB_HAND_EYE_TSAI,
        "PARK":       cv2.CALIB_HAND_EYE_PARK,
        "HORAUD":     cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF":    cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    print(f"\n{'':12} {' (mm)':<30} {'':<15} {''}")
    print("-" * 75)

    best_name = None
    best_error = float('inf')
    all_results = {}

    for name, method in methods.items():
        try:
            R_c2b, t_c2b = cv2.calibrateHandEye(
                R_base2gripper_list, t_base2gripper_list,
                R_target2cam_list, t_target2cam_list,
                method=method
            )

            T_c2b = np.eye(4)
            T_c2b[:3, :3] = R_c2b
            T_c2b[:3, 3] = t_c2b.flatten()

            det = np.linalg.det(R_c2b)
            ortho = np.linalg.norm(R_c2b @ R_c2b.T - np.eye(3))

            errors, _ = compute_consistency_error(
                R_gripper2base_list, t_gripper2base_list,
                R_target2cam_list, t_target2cam_list, T_c2b
            )
            mean_err = np.mean(errors) * 1000

            t_mm = t_c2b.flatten() * 1000
            marker = " <--" if method == HAND_EYE_METHOD else ""
            print(f"  {name:<12} "
                  f"[{t_mm[0]:7.2f}, {t_mm[1]:7.2f}, {t_mm[2]:7.2f}]  "
                  f"det={det:.4f}  "
                  f"{mean_err:.2f} mm{marker}")

            all_results[name] = T_c2b
            np.savetxt(os.path.join(SAVE_DIR, f"T_cam2base_{name}.txt"), T_c2b)

            if mean_err < best_error:
                best_error = mean_err
                best_name = name

        except Exception as e:
            print(f"  {name:<12} : {e}")

    if not all_results:
        print("\n!")
        return

    #
    primary_name = [k for k, v in methods.items() if v == HAND_EYE_METHOD][0]
    T_cam2base = all_results.get(primary_name, all_results.get(best_name))
    T_base2cam = np.linalg.inv(T_cam2base)

    np.savetxt(os.path.join(SAVE_DIR, "T_cam2base.txt"), T_cam2base)
    np.savetxt(os.path.join(SAVE_DIR, "T_base2cam.txt"), T_base2cam)

    # ---- 6.  ----
    print(f"\n[5/5] ...")
    print(f"  : {primary_name if primary_name in all_results else best_name}")
    print(f"  : {best_name} ({best_error:.2f} mm)")

    errors, mean_target_pos = compute_consistency_error(
        R_gripper2base_list, t_gripper2base_list,
        R_target2cam_list, t_target2cam_list, T_cam2base
    )
    print(f"\n  :")
    print(f"    : {np.mean(errors)*1000:.2f} mm")
    print(f"    : {np.max(errors)*1000:.2f} mm")
    print(f"    : {np.std(errors)*1000:.2f} mm")
    print(f"    : [{mean_target_pos[0]:.4f}, {mean_target_pos[1]:.4f}, {mean_target_pos[2]:.4f}] m")

    rpy = Rotation.from_matrix(T_cam2base[:3, :3]).as_euler('xyz', degrees=True)
    t = T_cam2base[:3, 3] * 1000
    print(f"\n  T_cam2base (->):")
    print(f"     : [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] mm")
    print(f"     : [{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}] deg")

    print(f"\n  T_cam2base (4x4):")
    for row in T_cam2base:
        print(f"    [{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}, {row[3]:10.6f}]")

    config = {
        "timestamp": datetime.now().isoformat(),
        "robot_ip": ROBOT_IP,
        "camera": "ZED 2",
        "calibration_type": "eye_to_hand",
        "checkerboard": f"{CHECKERBOARD_COLS}x{CHECKERBOARD_ROWS}",
        "square_size_m": SQUARE_SIZE,
        "num_valid_poses": valid_count,
        "method": primary_name if primary_name in all_results else best_name,
        "best_method": best_name,
        "mean_consistency_error_mm": float(np.mean(errors) * 1000),
        "max_consistency_error_mm": float(np.max(errors) * 1000),
    }
    with open(os.path.join(SAVE_DIR, "calibration_config.json"), 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    print(f"\n {SAVE_DIR}/")
    print("=" * 60)
    print("")


# ================================================================
# :
# ================================================================

def use_calibration_result(camera_point, T_cam2base):
    """
    (Eye-to-Hand)

    Args:
        camera_point: [x, y, z] ()
        T_cam2base: 4x4 ->  (T_cam2base.txt)

    Returns:
        [x, y, z]
    """
    p_cam = np.array([camera_point[0], camera_point[1], camera_point[2], 1.0])
    p_base = T_cam2base @ p_cam
    return p_base[:3]


def use_calibration_with_depth(pixel_u, pixel_v, depth_z, T_cam2base, camera_matrix):
    """
    (Eye-to-Hand)

    Args:
        pixel_u, pixel_v:
        depth_z: ()
        T_cam2base: 4x4 ->
        camera_matrix: 3x3

    Returns:
        [x, y, z]
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    x_cam = (pixel_u - cx) * depth_z / fx
    y_cam = (pixel_v - cy) * depth_z / fy
    z_cam = depth_z

    return use_calibration_result([x_cam, y_cam, z_cam], T_cam2base)


def load_calibration(save_dir=SAVE_DIR):
    """
    Returns: T_cam2base (4x4 numpy array), camera_matrix (3x3 numpy array)
    """
    T_path = os.path.join(save_dir, "T_cam2base.txt")
    cam_path = os.path.join(save_dir, "camera_matrix.txt")
    return np.loadtxt(T_path), np.loadtxt(cam_path)


if __name__ == '__main__':
    main()
