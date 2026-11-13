#!/bin/bash
#
# 一键在 rosbag2（.db3）数据集上回放 myslam VIO，或对数据集做体检。
#
# 与 run_subt.sh 同一套约定：默认构建 Release、用 config/config.lua 的
# vio.* / dataset.extrinsic 参数，脚本只负责「构建 + 启动」。
# 不同点：bag 的图像内参优先从 bag 自带的 camera_info 读取，缺失时才回退到
# Lua 配置，因此 SubT 的 640x480 MEI 标定不会悄悄套到 1920x1440 的 bag 上。
#
# 示例：
#   scripts/run_rosbag.sh                                   # 默认 07_25 包，跑 VIO（带窗口）
#   scripts/run_rosbag.sh /path/to/xxx.db3                  # 指定 bag 或包目录
#   scripts/run_rosbag.sh --inspect                         # 只做数据体检，不跑 VIO
#   scripts/run_rosbag.sh --no-show --video out.avi         # 不开窗口，只录像
#   scripts/run_rosbag.sh --max-frames 100 --plot           # 跑 100 帧并画轨迹
#   scripts/run_rosbag.sh --playback-rate 2                 # 两倍速播放
#   scripts/run_rosbag.sh --topic /camera/left/image_raw/compressed --fx 800 --fy 800
#   scripts/run_rosbag.sh --no-build                        # 跳过构建
#   DATA_ROOT=/data scripts/run_rosbag.sh                   # 换数据根目录
#
# 图像尺寸与内参不符时程序会直接终止：那说明 bag 没有 camera_info，
# 需要用 --fx/--fy/--cx/--cy 显式给标定，脚本不会替你猜。
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-release}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
# 1 = 从 thirdparty/opencv 子模块构建 OpenCV（默认用系统包）
OPENCV_BUNDLED="${OPENCV_BUNDLED:-0}"
DATA_ROOT="${MYSLAM_DATA_ROOT:-$ROOT/../data}"
CONFIG="${MYSLAM_CONFIG:-$ROOT/config/config.lua}"
# 默认选唯一带 camera_info 的包：开箱即可跑视觉，无需外部标定
DEFAULT_BAG="$DATA_ROOT/rosbag2_2026_07_25-19_05_21"

MODE=vio
DO_BUILD=1
PLOT=0
OUT=""
BAG=""
PASSTHRU=()

usage() {
    sed -n '2,23p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

可选参数:
  --inspect           只运行数据体检程序（3_rosbag_inspect），不跑 VIO
  --config FILE       使用指定的 Lua 配置（默认 config/config.lua）
  --out FILE.tum      轨迹输出（默认 <BUILD_DIR>/rosbag_<包名>.tum）
  --plot              回放结束后用 matplotlib 画轨迹图
  --no-build          跳过配置与编译，直接运行已有二进制
  -h, --help          显示本帮助

以下参数原样透传给 VIO 程序:
  --topic NAME  --start-frame N  --max-frames N  --stride N
  --max-features N  --max-duration SEC
  --fx V  --fy V  --cx V  --cy V
  --show / --no-show      开/关实时窗口（默认读 Lua dataset.viz，config.lua 里为开）
  --video FILE            录像（如 out.avi）
  --video-fps N           录像帧率（默认跟随相机）
  --playback-rate R       播放倍速（开窗口时默认 1 = 实时）

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
        --topic|--start-frame|--max-frames|--stride|--max-features|--max-duration|--fx|--fy|--cx|--cy|--video|--video-fps|--playback-rate)
            return 0 ;;
        *) return 1 ;;
    esac
}

# 无值的透传开关
is_passthru_flag() {
    case "$1" in
        --show|--no-show)
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
            if [ -n "$BAG" ]; then
                echo "❌ 只能指定一个 bag 路径（已给出 $BAG）" >&2
                exit 2
            fi
            BAG="$1"
            shift
            ;;
    esac
done

# bag 路径在 cd 到仓库根之前解析成绝对路径
if [ -z "$BAG" ]; then
    BAG="$DEFAULT_BAG"
fi
if [ ! -e "$BAG" ]; then
    echo "❌ bag 路径不存在: $BAG" >&2
    echo "   可用包:" >&2
    ls -d "$DATA_ROOT"/rosbag2_* 2>/dev/null | sed 's/^/     /' >&2 || true
    exit 1
fi
BAG="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"

if [ "$MODE" = "vio" ] && [ ! -f "$CONFIG" ]; then
    echo "❌ 配置文件不存在: $CONFIG" >&2
    exit 1
fi

if [ "$MODE" = "inspect" ]; then
    TARGET=3_rosbag_inspect
else
    TARGET=3_rosbag_vio
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

    echo "==> 构建 $TARGET（首次会编译 GTSAM，可能耗时十几分钟）"
    cmake --build "$BUILD_DIR" --target "$TARGET" -j"$JOBS"
fi

BIN="$BUILD_DIR/bin/$TARGET"
if [ ! -x "$BIN" ]; then
    echo "❌ 未找到可执行文件: $BIN（去掉 --no-build 先构建）" >&2
    exit 1
fi

# 相对路径（lua 里的 dataset.* 与轨迹输出）按仓库根解析
cd "$ROOT"

if [ "$MODE" = "inspect" ]; then
    echo "==> 数据体检: $BAG"
    "$BIN" "$BAG"
    exit 0
fi

if [ -z "$OUT" ]; then
    OUT="$BUILD_DIR/rosbag_$(basename "$BAG").tum"
fi
mkdir -p "$(dirname "$OUT")"

echo "==> VIO 回放: $BAG"
echo "    配置: $CONFIG"
echo "    轨迹: $OUT"
"$BIN" "$BAG" --config "$CONFIG" --out "$OUT" ${PASSTHRU[@]+"${PASSTHRU[@]}"}

if [ "$PLOT" -eq 1 ]; then
    if [ ! -s "$OUT" ]; then
        echo "⚠️  轨迹文件为空: $OUT（回放是否未完成初始化？）" >&2
        exit 0
    fi
    echo "==> 绘制轨迹图"
    python3 "$ROOT/scripts/plot_tum.py" "$OUT" --save "${OUT%.tum}.png"
fi
