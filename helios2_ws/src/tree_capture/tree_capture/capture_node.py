#!/usr/bin/env python3
import copy
import threading
from pathlib import Path

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge

from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def sanitize(value: str) -> str:
    cleaned = value.strip().replace(" ", "_").lower()
    invalid = "\\/:*?\"<>|"
    return "".join(ch for ch in cleaned if ch not in invalid)


def build_tree_dir(
    base_dir: Path,
    camera_type: str,
    label: str,
    row: str,
    variety: str,
    tree_id_4: str,
) -> Path:
    """
    New structure:
      <base_dir>/<camera_type>/<label>/tree_<row>_<variety>_<tree_id%04d>/
    """
    label_folder = label
    variety_folder = variety
    tree_folder = f"tree_{row}_{variety_folder}_{tree_id_4}"
    return base_dir / camera_type / label_folder / tree_folder


def make_count_str(view_id: str) -> str:
    """Image count used in filenames (un-padded by default)."""
    try:
        return str(int(view_id))
    except Exception:
        return str(view_id)


def normalize_depth_u8(depth: np.ndarray) -> np.ndarray:
    """
    Normalize depth image to uint8 for preview/saving PNG heatmaps.
    Works with float depth (meters or millimeters) robustly.
    """
    depth = depth.astype(np.float32)
    valid = np.isfinite(depth) & (depth > 1e-6)
    if not np.any(valid):
        return np.zeros(depth.shape, dtype=np.uint8)

    d = depth[valid]
    lo = np.percentile(d, 1.0)
    hi = np.percentile(d, 99.0)
    if hi <= lo + 1e-9:
        hi = lo + 1.0

    out = (depth - lo) * (255.0 / (hi - lo))
    out = np.clip(out, 0.0, 255.0)
    out[~valid] = 0.0
    return out.astype(np.uint8)


def depth_m_to_u16mm(depth_m: np.ndarray) -> np.ndarray:
    d = depth_m.astype(np.float32)
    valid = np.isfinite(d) & (d > 1e-6)

    out = np.zeros(d.shape, dtype=np.uint16)
    if np.any(valid):
        mm_valid = np.clip(d[valid] * 1000.0, 0.0, 65535.0).astype(np.uint16)
        out[valid] = mm_valid
    return out

def camera_info_to_dict(ci: CameraInfo) -> dict:
    return {
        "width": int(ci.width),
        "height": int(ci.height),
        "distortion_model": str(ci.distortion_model),
        "d": [float(x) for x in ci.d],
        "k": [float(x) for x in ci.k],
        "r": [float(x) for x in ci.r],
        "p": [float(x) for x in ci.p],
    }


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    PointCloud2 -> Nx3 float32 in meters, with NaN/Inf rows removed.
    """
    try:
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        pts = np.asarray(pts, dtype=np.float32).reshape((-1, 3))
    except Exception:
        it = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        pts = np.array(list(it), dtype=np.float32).reshape((-1, 3))

    # Drop any row containing NaN/Inf
    if pts.size == 0:
        return pts

    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    return pts


def write_ply_xyz(points: np.ndarray, path: Path, rotate_x_180: bool = True) -> None:
    if points.size == 0:
        print("Point cloud is empty; not writing PLY.")
        return

    pts = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.size == 0:
        print("Point cloud has only NaN/Inf; not writing PLY.")
        return

    ensure_dir(path.parent)

    if rotate_x_180:
        pts = pts.copy()
        pts[:, 1] = -pts[:, 1]
        pts[:, 2] = -pts[:, 2]

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {pts.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for x, y, z in pts:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


class TreeCaptureUINode(Node):
    def __init__(self):
        super().__init__("tree_capture")

        self.bridge = CvBridge()

        # ---------------- Parameters ----------------
        self.declare_parameter("base_dir", "BRANCH_vZ")
        self.declare_parameter("camera_type", "helios2")
        self.declare_parameter("label", "B")          # folder: before/after/etc (sanitized)
        self.declare_parameter("row", "1")                 # row
        self.declare_parameter("variety", "unknown")       # used in tree folder name (sanitized)
        self.declare_parameter("tree_id", "0000")          # will be zfilled(4)
        self.declare_parameter("view_id", "0")             # image_count
        self.declare_parameter("capture_dir", "captures")  # subfolder under tree_.../

        self.declare_parameter("preview_width", 960)
        self.declare_parameter("preview_height", 540)

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("intensity_topic", "/camera/intensity/image_raw")
        self.declare_parameter("points_topic", "/camera/points")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # message_filters tuning
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.05)

        # saving options
        self.declare_parameter("rotate_ply_x180", True)

        # ---------------- Load params ----------------
        self.base_dir = Path(self.get_parameter("base_dir").value)
        self.camera_type = str(self.get_parameter("camera_type").value)
        self.label = str(self.get_parameter("label").value)
        self.row = str(self.get_parameter("row").value)
        self.variety = str(self.get_parameter("variety").value)
        self.tree_id = str(self.get_parameter("tree_id").value).zfill(4)
        self.view_id = str(self.get_parameter("view_id").value)
        self.capture_dir = str(self.get_parameter("capture_dir").value)

        self.preview_w = int(self.get_parameter("preview_width").value)
        self.preview_h = int(self.get_parameter("preview_height").value)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.intensity_topic = str(self.get_parameter("intensity_topic").value)
        self.points_topic = str(self.get_parameter("points_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop = float(self.get_parameter("sync_slop_sec").value)
        self.rotate_ply = bool(self.get_parameter("rotate_ply_x180").value)

        self.use_heatmap = False

        # Latest synchronized bundle
        self._lock = threading.Lock()
        self._last_sync = None  # tuple(depth_msg, intensity_msg, cloud_msg, caminfo_msg)

        # ---------------- Synchronized subscribers ----------------
        # IMPORTANT: use qos_profile_sensor_data to match your driver (best effort)
        self.sub_depth = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.sub_intensity = Subscriber(self, Image, self.intensity_topic, qos_profile=qos_profile_sensor_data)
        self.sub_cloud = Subscriber(self, PointCloud2, self.points_topic, qos_profile=qos_profile_sensor_data)
        self.sub_caminfo = Subscriber(self, CameraInfo, self.camera_info_topic, qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_intensity, self.sub_cloud, self.sub_caminfo],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._on_sync)

        self.get_logger().info("TreeCaptureUINode running (synchronized saving enabled).")
        self.get_logger().info(
            "Sync topics:\n"
            f"  depth:      {self.depth_topic}\n"
            f"  intensity:  {self.intensity_topic}\n"
            f"  points:     {self.points_topic}\n"
            f"  cam_info:   {self.camera_info_topic}\n"
            f"  slop: {self.sync_slop}s, queue: {self.sync_queue_size}"
        )

        # ---------------- OpenCV window ----------------
        self.window_name = "Tree Capture UI"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.preview_w, self.preview_h)
        self._update_title()

    def _on_sync(self, depth_msg: Image, intensity_msg: Image, cloud_msg: PointCloud2, caminfo_msg: CameraInfo):
        # store the newest synchronized set
        with self._lock:
            self._last_sync = (depth_msg, intensity_msg, cloud_msg, caminfo_msg)

    def _update_title(self):
        title = (
            f"Capture ({self.label}, row={self.row}, variety={self.variety}) | "
            f"tree={self.tree_id} view={self.view_id} | "
            f"{'HEATMAP' if self.use_heatmap else 'GRAYSCALE'} | "
            "SPACE=save, N=next tree, H=heatmap, ESC=exit"
        )
        try:
            cv2.setWindowTitle(self.window_name, title)
        except Exception:
            pass

    def render_preview(self):
        with self._lock:
            bundle = self._last_sync

        if bundle is None:
            return

        depth_msg = bundle[0]
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return

        depth = np.asarray(depth)
        u8 = normalize_depth_u8(depth)
        vis = u8 if not self.use_heatmap else cv2.applyColorMap(u8, cv2.COLORMAP_JET)
        vis = cv2.resize(vis, (self.preview_w, self.preview_h))
        cv2.imshow(self.window_name, vis)

    def handle_key(self, key: int):
        if key == 27:  # ESC
            raise KeyboardInterrupt

        if key in (ord("h"), ord("H")):
            self.use_heatmap = not self.use_heatmap
            self._update_title()
            return

        if key in (ord("n"), ord("N")):
            self.tree_id = str(int(self.tree_id) + 1).zfill(4)
            self.view_id = "0"
            self._update_title()
            return

        if key == 32:  # SPACE
            with self._lock:
                bundle = self._last_sync

            if bundle is None:
                self.get_logger().warn("No synchronized bundle received yet; cannot save.")
                return

            # Snapshot view id for this capture BEFORE launching the thread
            count = make_count_str(self.view_id)

            # Copy messages so saving isn't racing with new arrivals
            depth_msg = copy.deepcopy(bundle[0])
            intensity_msg = copy.deepcopy(bundle[1])
            cloud_msg = copy.deepcopy(bundle[2])
            caminfo_msg = copy.deepcopy(bundle[3])

            threading.Thread(
                target=self._save_bundle,
                args=(depth_msg, intensity_msg, cloud_msg, caminfo_msg, count),
                daemon=True,
            ).start()

            # Now increment for the next capture
            self.view_id = str(int(self.view_id) + 1)
            self._update_title()

    def _save_bundle(
        self,
        depth_msg: Image,
        intensity_msg: Image,
        cloud_msg: PointCloud2,
        caminfo_msg: CameraInfo,
        count: str,
    ):
        tree_dir = build_tree_dir(
            self.base_dir,
            self.camera_type,
            self.label,
            self.row,
            self.variety,
            self.tree_id,
        )

        capture_root = tree_dir / self.capture_dir
        depth_dir = capture_root / "depth"
        pc_dir = capture_root / "pointcloud"
        ensure_dir(depth_dir)
        ensure_dir(pc_dir)

        depth_png = depth_dir / f"{count}_depth.png"
        heat_png  = depth_dir / f"{count}_heatmap.png"
        inten_png = depth_dir / f"{count}_intensity.png"
        cam_yaml  = depth_dir / f"{count}_camera_params.yaml"
        ply_path  = pc_dir / f"{count}.ply"

        # Depth: save as 16-bit PNG in millimeters (from float meters)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.asarray(depth).astype(np.float32)
        depth_u16 = depth_m_to_u16mm(depth)
        cv2.imwrite(str(depth_png), depth_u16)

        # Heatmap: based on robust 8-bit normalization
        u8 = normalize_depth_u8(depth)
        cv2.imwrite(str(heat_png), cv2.applyColorMap(u8, cv2.COLORMAP_JET))

        # Intensity (save as-is; MONO16 will be 16-bit PNG)
        try:
            inten = self.bridge.imgmsg_to_cv2(intensity_msg, desired_encoding="passthrough")
            inten = np.asarray(inten)
            cv2.imwrite(str(inten_png), inten)
        except Exception:
            pass

        # CameraInfo -> YAML
        try:
            with open(cam_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(camera_info_to_dict(caminfo_msg), f, sort_keys=False)
        except Exception as e:
            self.get_logger().warn(f"CameraInfo save failed: {e}")

        # Point cloud -> PLY
        try:
            pts = pointcloud2_to_xyz(cloud_msg)
            write_ply_xyz(pts, ply_path, rotate_x_180=self.rotate_ply)
        except Exception as e:
            self.get_logger().warn(f"PLY save failed: {e}")

        self.get_logger().info(
            "Saved:\n"
            f"  {depth_png}\n"
            f"  {heat_png}\n"
            f"  {inten_png}\n"
            f"  {cam_yaml}\n"
            f"  {ply_path}"
        )

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TreeCaptureUINode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.render_preview()
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
