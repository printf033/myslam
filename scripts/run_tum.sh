#!/bin/bash
#
# 一键在 TUM RGB-D 数据集上回放 myslam VIO，并输出与真值的轨迹误差（ATE）。
#
# 与 run_subt.sh / run_rosbag.sh 同一套约定：默认构建 Release、用
# config/config.lua 的 vio.* 参数，脚本只负责「构建 + 启动」。
#
# TUM 的独特之处是**自带运动捕捉真值**，所以回放结束会自动算 ATE；
# 但 TUM RGB-D 没有陀螺仪，gyro 由真值姿态差分合成，结果验证的是
# 位置融合链路，不能等同于标准 VIO 的精度结论。
#
# 示例：
#   scripts/run_tum.sh                                  # 默认 desk 序列
#   scripts/run_tum.sh /path/to/rgbd_dataset_freiburg1_xyz
#   scripts/run_tum.sh --inspect                        # 只做数据体检
#   scripts/run_tum.sh --max-frames 100 --plot          # 跑 100 帧并画轨迹
#   scripts/run_tum.sh --no-build                       # 跳过构建
#   DATA_ROOT=/data scripts/run_tum.sh                  # 换数据根目录
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-release}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
OPENCV_BUNDLED="${OPENCV_BUNDLED:-0}"
DATA_ROOT="${MYSLAM_DATA_ROOT:-$ROOT/../data}"
CONFIG="${MYSLAM_CONFIG:-$ROOT/config/config.lua}"
DEFAULT_SEQUENCE="$DATA_ROOT/tum/rgbd_dataset_freiburg1_desk"

MODE=vio
DO_BUILD=1
PLOT=0
OUT=""
SEQUENCE=""
PASSTHRU=()

usage() {
    sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

可选参数:
  --inspect           只运行数据体检，不跑 VIO
  --config FILE       使用指定的 Lua 配置（默认 config/config.lua）
  --out FILE.tum      轨迹输出（默认 <BUILD_DIR>/tum_<序列名>.tum）
  --plot              回放结束后用 matplotlib 画轨迹图
  --no-build          跳过配置与编译，直接运行已有二进制
  -h, --help          显示本帮助

以下参数原样透传给 VIO 程序:
  --start-frame N  --max-frames N  --stride N  --max-features N  --max-duration S
  --max-time-diff S   真值关联阈值（默认 0.02，与 TUM 官方一致）
  --fx V  --fy V  --cx V  --cy V
  --keep-imu-config   保留 Lua 的 IMU 噪声参数（默认按 TUM 实测值适配）
  --no-undistort      关闭图像去畸变（默认按标定矫正）
  --show / --no-show  开/关实时窗口
  --video FILE  --video-fps N  --playback-rate R

构建相关用环境变量控制:
  BUILD_DIR          CMake 构建目录（默认 ./build-release）
  BUILD_TYPE         Release（默认）或 Debug
  JOBS               并行编译任务数（默认 nproc）
  OPENCV_BUNDLED     1 = 从 thirdparty/opencv 构建 OpenCV
  DATA_ROOT          数据根目录（默认 ../data，相对仓库根）
EOF
}

# 需要取值的透传参数
is_passthru_with_value() {
    case "$1" in
        --start-frame|--max-frames|--stride|--max-features|--max-duration|--max-time-diff|--fx|--fy|--cx|--cy|--video|--video-fps|--playback-rate)
            return 0 ;;
        *) return 1 ;;
    esac
}

# 无值的透传开关
is_passthru_flag() {
    case "$1" in
        --show|--no-show|--no-undistort|--keep-imu-config)
            return 0 ;;
        *) return 1 ;;
    esac
}

while [ $# -gt 0 ]; do
    case "$1" in
        --inspect)  MODE=inspect; shift ;;
        --config)   CONFIG="$2"; shift 2 ;;
        --out)      OUT="$2"; shift 2 ;;
        --plot)     PLOT=1; shift ;;
        --no-build) DO_BUILD=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        -*)
            if is_passthru_with_value "$1"; then
                if [ $# -lt 2 ]; then
                    echo "❌ $1 缺少参数值" >&2
                    exit 2
                fi
                PASSTHRU+=("$1" "$2")
                shift 2
            elif is_passthru_flag "$1"; then
                PASSTHRU+=("$1")
                shift
            else
                echo "未知参数: $1" >&2
                usage >&2
                exit 2
            fi
            ;;
        *)
            if [ -n "$SEQUENCE" ]; then
                echo "❌ 只能指定一个序列目录（已给出 $SEQUENCE）" >&2
                exit 2
            fi
            SEQUENCE="$1"
            shift
            ;;
    esac
done

if [ -z "$SEQUENCE" ]; then
    SEQUENCE="$DEFAULT_SEQUENCE"
fi
if [ ! -d "$SEQUENCE" ]; then
    echo "❌ 序列目录不存在: $SEQUENCE" >&2
    echo "   可用序列:" >&2
    ls -d "$DATA_ROOT"/tum/rgbd_dataset_* 2>/dev/null | sed 's/^/     /' >&2 || true
    exit 1
fi
SEQUENCE="$(cd "$SEQUENCE" && pwd)"

if [ "$MODE" = "vio" ] && [ ! -f "$CONFIG" ]; then
    echo "❌ 配置文件不存在: $CONFIG" >&2
    exit 1
fi

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

    echo "==> 构建 4_tum_vio（首次会编译 GTSAM，可能耗时十几分钟）"
    cmake --build "$BUILD_DIR" --target 4_tum_vio -j"$JOBS"
fi

BIN="$BUILD_DIR/bin/4_tum_vio"
if [ ! -x "$BIN" ]; then
    echo "❌ 未找到可执行文件: $BIN（去掉 --no-build 先构建）" >&2
    exit 1
fi

# 相对路径（lua 里的 dataset.* 与轨迹输出）按仓库根解析
cd "$ROOT"

if [ "$MODE" = "inspect" ]; then
    echo "==> 数据体检: $SEQUENCE"
    "$BIN" "$SEQUENCE" --inspect
    exit 0
fi

if [ -z "$OUT" ]; then
    OUT="$BUILD_DIR/tum_$(basename "$SEQUENCE").tum"
fi
mkdir -p "$(dirname "$OUT")"

echo "==> VIO 回放: $SEQUENCE"
echo "    配置: $CONFIG"
echo "    轨迹: $OUT"
"$BIN" "$SEQUENCE" --config "$CONFIG" --out "$OUT" ${PASSTHRU[@]+"${PASSTHRU[@]}"}

if [ "$PLOT" -eq 1 ]; then
    if [ ! -s "$OUT" ]; then
        echo "⚠️  轨迹文件为空: $OUT（回放是否未完成初始化？）" >&2
        exit 0
    fi
    echo "==> 绘制轨迹图"
    python3 "$ROOT/scripts/plot_tum.py" "$OUT" --save "${OUT%.tum}.png"
fi
