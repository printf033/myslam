#!/bin/bash
#
# 在 SubT-MRS 数据集上回放 myslam VIO。
#
# 默认使用 MASt3R-Fusion 仓库里的 Handheld1 序列（项目内的 dataset 软链接
# 指向 ../data/MASt3R-Fusion/dataset）。前段图像曝光剧烈跳变且 IMU 存在
# 0.4s 空洞，因此默认从第 2300 帧开始（与 MASt3R-Fusion 的 --start_from 一致）。
#
# 脚本默认使用 Release + 关闭 sanitizer 的独立构建目录 build-release：
# Debug/ASan 下每次关键帧优化需要数十秒，不适合完整回放。
#
# 示例：
#   scripts/run_subt.sh                              # 从 2300 帧起跑完剩余全部帧
#   scripts/run_subt.sh --max-frames 500             # 只跑前 500 帧（快速预览）
#   scripts/run_subt.sh --start-frame 0 --max-frames 300
#   scripts/run_subt.sh --root /path/Handheld2_Folder
#   scripts/run_subt.sh --viz                        # 实时窗口（ESC/q 退出）
#   scripts/run_subt.sh --video out.avi --ply cloud.ply
#   scripts/run_subt.sh --max-frames 0 --viz --viz-rate 1   # 全程实时播放
#   OPENCV_BUNDLED=1 scripts/run_subt.sh            # 用 thirdparty/opencv 子模块构建 OpenCV
#   scripts/run_subt.sh --init mast3r               # 用 MASt3R 估计内参，无需手填相机参数
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-release}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
DATA_ROOT="${SUBT_ROOT:-$ROOT/../MASt3R-Fusion/dataset/Handheld1_Folder}"
START_FRAME="${START_FRAME:-2300}"
MAX_FRAMES="${MAX_FRAMES:-0}"
STRIDE="${STRIDE:-1}"
MAX_FEATURES="${MAX_FEATURES:-200}"
OUTPUT="${OUTPUT:-}"
VIDEO="${VIDEO:-}"
PLY="${PLY:-}"
VIZ="${VIZ:-0}"
VIZ_STRIDE="${VIZ_STRIDE:-1}"
VIZ_RATE="${VIZ_RATE:-0}"
PLOT="${PLOT:-0}"
# 1 = 从 thirdparty/opencv 子模块构建 OpenCV（默认用系统包）
OPENCV_BUNDLED="${OPENCV_BUNDLED:-0}"
# 内参来源：空 = 用配置/内置默认值；mast3r = 从图像自动估计
INIT_MODE="${INIT_MODE:-}"
# 跑 MASt3R 估计的 Python 解释器（需装 torch/dust3r）
MAST3R_PYTHON="${MAST3R_PYTHON:-${MYSLAM_PYTHON:-}}"
FORCE_ESTIMATE="0"
DO_BUILD=1

usage() {
    sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

可选参数:
  --root DIR         序列目录（含 cam_0/ 与 imu/），默认 Handheld1_Folder
  --start-frame N    从第 N 帧开始（默认 2300）
  --max-frames N     最多处理帧数，0 表示全部（默认 0）
  --stride N         每隔 N 帧取一帧（默认 1）
  --max-features N   每帧特征数上限（默认 200）
  --output FILE      轨迹输出路径（TUM），默认 build-release/subt_轨迹名.tum
  --viz              打开实时可视化窗口（需要 DISPLAY，ESC/q 退出）
  --video FILE       把可视化画面写入视频（mp4 不支持时自动回退 .avi/MJPG）
  --viz-stride N     每 N 帧渲染一次（默认 1）
  --viz-rate R       播放倍速：0=全速（默认）、1=实时、2=两倍速
  --ply FILE         导出路标+轨迹点云，可用 MeshLab/CloudCompare 打开
  --plot             回放结束后用 matplotlib 画出轨迹 PNG
  --build-dir DIR    CMake 构建目录（默认 ./build-release）
  --build-type T     Release（默认）或 Debug
  --init MODE        manual（默认，用配置/内置参数）或 mast3r（从图像估计内参）
  --force-estimate   忽略 MASt3R 内参缓存，重新估计
  --no-build         跳过配置与编译，直接运行已有二进制
  -h, --help         显示本帮助
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --root)         DATA_ROOT="$2"; shift 2 ;;
        --start-frame)  START_FRAME="$2"; shift 2 ;;
        --max-frames)   MAX_FRAMES="$2"; shift 2 ;;
        --stride)       STRIDE="$2"; shift 2 ;;
        --max-features) MAX_FEATURES="$2"; shift 2 ;;
        --output)       OUTPUT="$2"; shift 2 ;;
        --video)        VIDEO="$2"; shift 2 ;;
        --ply)          PLY="$2"; shift 2 ;;
        --viz-stride)   VIZ_STRIDE="$2"; shift 2 ;;
        --viz-rate)     VIZ_RATE="$2"; shift 2 ;;
        --viz)          VIZ=1; shift ;;
        --plot)         PLOT=1; shift ;;
        --build-dir)    BUILD_DIR="$2"; shift 2 ;;
        --build-type)   BUILD_TYPE="$2"; shift 2 ;;
        --init)         INIT_MODE="$2"; shift 2 ;;
        --force-estimate) FORCE_ESTIMATE=1; shift ;;
        --no-build)     DO_BUILD=0; shift ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
    esac
done

CAM_DIR="$DATA_ROOT/cam_0"
IMU_FILE="$DATA_ROOT/imu/imu_data.csv"
STAMP_FILE="$CAM_DIR/timestamps.txt"

missing=0
for f in "$CAM_DIR" "$IMU_FILE" "$STAMP_FILE"; do
    if [ ! -e "$f" ]; then
        echo "❌ 缺少数据路径: $f" >&2
        missing=1
    fi
done
if [ "$missing" -ne 0 ]; then
    echo "   请用 --root 或 SUBT_ROOT 指向包含 cam_0/ 与 imu/ 的序列目录" >&2
    exit 1
fi

if [ -z "$OUTPUT" ]; then
    seq_name="$(basename "$DATA_ROOT")"
    OUTPUT="$BUILD_DIR/subt_${seq_name}.tum"
fi
mkdir -p "$(dirname "$OUTPUT")"

if [ "$DO_BUILD" -eq 1 ]; then
    echo "==> 配置 CMake ($BUILD_TYPE, MYSLAM_BUILD_DATASET_TOOLS=ON)"
    configure_flags=(
        -B "$BUILD_DIR" -G Ninja
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        -DMYSLAM_BUILD_TESTS=OFF
        -DMYSLAM_BUILD_EXAMPLES=ON
        -DMYSLAM_BUILD_DATASET_TOOLS=ON
    )
    if [ "$BUILD_TYPE" = "Release" ]; then
        configure_flags+=(-DMYSLAM_SANITIZE=OFF)
    fi
    if [ "$OPENCV_BUNDLED" = "1" ]; then
        configure_flags+=(-DMYSLAM_USE_BUNDLED_OPENCV=ON)
    fi
    cmake "${configure_flags[@]}"

    echo "==> 构建 2_subt_dataset（首次会编译 GTSAM，可能耗时十几分钟）"
    cmake --build "$BUILD_DIR" --target 2_subt_dataset -j"$JOBS"
fi

BIN="$BUILD_DIR/bin/2_subt_dataset"
if [ ! -x "$BIN" ]; then
    echo "❌ 未找到可执行文件: $BIN（去掉 --no-build 先构建）" >&2
    exit 1
fi

echo "==> 回放序列: $DATA_ROOT"
echo "    起始帧: $START_FRAME, 帧上限: $MAX_FRAMES (0=全部), stride: $STRIDE, 特征上限: $MAX_FEATURES"
echo "    轨迹输出: $OUTPUT"
[ -n "$VIDEO" ] && echo "    可视化视频: $VIDEO"
[ -n "$PLY" ] && echo "    点云输出:   $PLY"
[ -n "$INIT_MODE" ] && echo "    内参来源:   $INIT_MODE"
[ "$VIZ" -eq 1 ] && echo "    实时窗口:   开启（ESC/q 退出），倍速 ${VIZ_RATE}（0=全速）"

cd "$ROOT"
run_args=(
    --data "$CAM_DIR"
    --imu "$IMU_FILE"
    --stamps "$STAMP_FILE"
    --start-frame "$START_FRAME"
    --max-frames "$MAX_FRAMES"
    --stride "$STRIDE"
    --max-features "$MAX_FEATURES"
    --output "$OUTPUT"
)
[ -n "$VIDEO" ] && run_args+=(--video "$VIDEO" --viz-stride "$VIZ_STRIDE")
[ "$VIZ" -eq 1 ] && run_args+=(--viz-rate "$VIZ_RATE")
[ -n "$PLY" ] && run_args+=(--ply "$PLY")
[ "$VIZ" -eq 1 ] && run_args+=(--viz)
[ -n "$INIT_MODE" ] && run_args+=(--init "$INIT_MODE")
[ "$FORCE_ESTIMATE" -eq 1 ] && run_args+=(--force-estimate)
# MASt3R 推理用的解释器：C++ 侧读 MYSLAM_PYTHON
[ -n "$MAST3R_PYTHON" ] && export MYSLAM_PYTHON="$MAST3R_PYTHON"
"$BIN" "${run_args[@]}"

if [ "$PLOT" -eq 1 ]; then
    echo "==> 绘制轨迹图"
    plot_args=("$OUTPUT" --save "${OUTPUT%.tum}.png")
    [ -n "$PLY" ] && plot_args+=(--ply "$PLY")
    python3 "$ROOT/scripts/plot_tum.py" "${plot_args[@]}"
fi
