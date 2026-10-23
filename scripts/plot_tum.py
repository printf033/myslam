#!/usr/bin/env python3
"""绘制 myslam 回放输出的轨迹（TUM 格式），可选叠加 PLY 点云。

用法:
    scripts/plot_tum.py build-release/subt_Handheld1_Folder.tum
    scripts/plot_tum.py traj.tum --ply cloud.ply --save traj.png --show
    scripts/plot_tum.py traj.tum --save traj.png        # 只存文件

输出三张子图：3D 轨迹、XY 俯视、位置/速度随时间变化。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> np.ndarray:
    """读取 TUM 轨迹，返回 (N, 8) 数组：t tx ty tz qx qy qz qw。"""
    rows = []
    with path.open() as handle:
        for lineno, line in enumerate(handle, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 8:
                print(f"[warn] 第 {lineno} 行字段数为 {len(fields)}，跳过", file=sys.stderr)
                continue
            try:
                rows.append([float(v) for v in fields])
            except ValueError:
                print(f"[warn] 第 {lineno} 行无法解析，跳过", file=sys.stderr)
    if not rows:
        raise SystemExit(f"没有从 {path} 读到有效轨迹")
    return np.asarray(rows)


def load_ply_vertices(path: Path, limit: int = 20000) -> np.ndarray:
    """读取 ASCII PLY 的前 limit 个顶点坐标。"""
    with path.open() as handle:
        count = 0
        header_done = False
        points = []
        for line in handle:
            if not header_done:
                if line.startswith("element vertex"):
                    count = int(line.split()[-1])
                elif line.strip() == "end_header":
                    header_done = True
                continue
            if len(points) >= limit:
                break
            parts = line.split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        vertices = np.asarray(points) if points else np.empty((0, 3))
    if count:
        print(f"[info] PLY 共 {count} 个顶点，绘制前 {len(vertices)} 个", file=sys.stderr)
    return vertices


def main() -> int:
    parser = argparse.ArgumentParser(description="绘制 TUM 轨迹")
    parser.add_argument("tum", type=Path, help="TUM 轨迹文件")
    parser.add_argument("--ply", type=Path, help="可选：叠加 PLY 点云")
    parser.add_argument("--save", type=Path, help="输出 PNG 路径")
    parser.add_argument("--show", action="store_true", help="交互窗口显示")
    parser.add_argument("--equal", action="store_true", default=True, help="3D 轴等比例（默认开启）")
    args = parser.parse_args()

    if not args.save and not args.show:
        args.save = args.tum.with_suffix(".png")

    import matplotlib

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = load_tum(args.tum)
    t, xyz, quat = data[:, 0], data[:, 1:4], data[:, 4:8]

    # 时间相对化，便于阅读
    t = t - t[0]
    vel = np.zeros_like(xyz)
    if len(t) > 1:
        dt = np.diff(t)
        dt[dt <= 1e-9] = 1e-9
        vel[1:] = np.diff(xyz, axis=0) / dt[:, None]
    speed = np.linalg.norm(vel, axis=1)

    fig = plt.figure(figsize=(15, 5))

    # --- 3D 轨迹 ---
    ax3d = fig.add_subplot(131, projection="3d")
    if args.ply and args.ply.exists():
        cloud = load_ply_vertices(args.ply)
        if len(cloud):
            stride = max(1, len(cloud) // 8000)
            ax3d.scatter(cloud[::stride, 0], cloud[::stride, 1], cloud[::stride, 2],
                         s=1, c="#4fc3f7", alpha=0.35, label="landmarks")
    ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], c="#e53935", lw=1.6, label="trajectory")
    ax3d.scatter(*xyz[0], c="#43a047", s=45, marker="o", label="start")
    ax3d.scatter(*xyz[-1], c="#1e88e5", s=45, marker="^", label="end")
    if args.equal:
        span = max(np.ptp(xyz[:, 0]), np.ptp(xyz[:, 1]), np.ptp(xyz[:, 2]), 1.0) * 0.55
        mid = xyz.mean(axis=0)
        ax3d.set_xlim(mid[0] - span, mid[0] + span)
        ax3d.set_ylim(mid[1] - span, mid[1] + span)
        ax3d.set_zlim(mid[2] - span, mid[2] + span)
        ax3d.set_box_aspect((1, 1, 1))
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.set_title(f"3D trajectory ({len(xyz)} keyframes)")
    ax3d.legend(loc="upper left", fontsize=8)

    # --- XY 俯视 ---
    ax2d = fig.add_subplot(132)
    ax2d.plot(xyz[:, 0], xyz[:, 1], c="#e53935", lw=1.6)
    ax2d.scatter(xyz[0, 0], xyz[0, 1], c="#43a047", s=45, marker="o", label="start")
    ax2d.scatter(xyz[-1, 0], xyz[-1, 1], c="#1e88e5", s=45, marker="^", label="end")
    ax2d.set_aspect("equal", adjustable="datalim")
    ax2d.grid(alpha=0.3)
    ax2d.set_xlabel("x [m]")
    ax2d.set_ylabel("y [m]")
    ax2d.set_title("top view (XY)")
    ax2d.legend(fontsize=8)

    # --- 位置 / 速度 ---
    ax_t = fig.add_subplot(133)
    ax_t.plot(t, xyz[:, 0], label="x", lw=1.2)
    ax_t.plot(t, xyz[:, 1], label="y", lw=1.2)
    ax_t.plot(t, xyz[:, 2], label="z", lw=1.2)
    ax_t.set_xlabel("time [s]")
    ax_t.set_ylabel("position [m]")
    ax_t.grid(alpha=0.3)
    ax_t.legend(loc="upper left", fontsize=8)
    ax_v = ax_t.twinx()
    ax_v.plot(t, speed, c="0.35", ls="--", lw=1.0, label="speed")
    ax_v.set_ylabel("speed [m/s]")
    ax_t.set_title("position and speed")

    path_span = np.linalg.norm(xyz[-1] - xyz[0])
    fig.suptitle(
        f"myslam replay: {len(xyz)} keyframes, {t[-1]:.1f}s, "
        f"path {np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)):.1f}m, "
        f"start-end {path_span:.1f}m"
    )
    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=130)
        print(f"[info] 已保存 {args.save}", file=sys.stderr)
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
