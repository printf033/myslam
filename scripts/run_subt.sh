#!/bin/bash
#
# 在 SubT-MRS 数据集上回放 myslam VIO。
#
# 运行时配置全部在 config/subt_handheld1.lua（见文件里的 dataset 一节）：
#   序列目录、内参来源、回放起止帧、轨迹/点云输出、可视化开关、算法参数。
# 本脚本只负责「构建 + 启动」，不再提供与 Lua 字段一一对应的一套命令行参数
# ——那套参数每加一个字段都要改三处，而且漏传时配置会被静默地当成默认值。
#
# 示例：
#   scripts/run_subt.sh                       # 构建并回放（用 config/subt_handheld1.lua）
#   scripts/run_subt.sh --no-build            # 跳过构建，直接运行已有二进制
#   scripts/run_subt.sh --config config/ablation_no_gravity.lua   # 换一份配置
#   scripts/run_subt.sh --plot                # 回放结束后画轨迹图
#   OPENCV_BUNDLED=1 scripts/run_subt.sh      # 用 thirdparty/opencv 子模块构建 OpenCV
#   BUILD_DIR=build scripts/run_subt.sh       # 换构建目录
#
# 消融实验请复制一份 lua 再改（例如 vio.gravityAlignment = false），
# 不要指望脚本帮你拼命令行覆盖项。
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-release}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
# 1 = 从 thirdparty/opencv 子模块构建 OpenCV（默认用系统包）
OPENCV_BUNDLED="${OPENCV_BUNDLED:-0}"
CONFIG="${MYSLAM_CONFIG:-$ROOT/config/subt_handheld1.lua}"
DO_BUILD=1
PLOT=0

usage() {
    sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

可选参数:
  --config FILE      使用指定的 Lua 配置（默认 config/subt_handheld1.lua）
  --plot             回放结束后用 matplotlib 画轨迹图（读配置里的输出路径）
  --no-build         跳过配置与编译，直接运行已有二进制
  -h, --help         显示本帮助

构建相关用环境变量控制:
  BUILD_DIR          CMake 构建目录（默认 ./build-release）
  BUILD_TYPE         Release（默认）或 Debug
  JOBS               并行编译任务数（默认 nproc）
  OPENCV_BUNDLED     1 = 从 thirdparty/opencv 构建 OpenCV

所有运行时参数（序列路径、起止帧、输出、可视化、算法）都在 Lua 配置里。
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2"; shift 2 ;;
        --plot)     PLOT=1; shift ;;
        --no-build) DO_BUILD=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "未知参数: $1（运行时配置请写进 $(basename "$CONFIG")）" >&2; usage >&2; exit 2 ;;
    esac
done

if [ ! -f "$CONFIG" ]; then
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

    echo "==> 构建 2_subt_dataset（首次会编译 GTSAM，可能耗时十几分钟）"
    cmake --build "$BUILD_DIR" --target 2_subt_dataset -j"$JOBS"
fi

BIN="$BUILD_DIR/bin/2_subt_dataset"
if [ ! -x "$BIN" ]; then
    echo "❌ 未找到可执行文件: $BIN（去掉 --no-build 先构建）" >&2
    exit 1
fi

# 相对路径（含 lua 里的 dataset.sequence.root）按仓库根解析
cd "$ROOT"
echo "==> 回放配置: $CONFIG"
"$BIN" --config "$CONFIG"

if [ "$PLOT" -eq 1 ]; then
    # 从 lua 里取输出路径：单行 `key = "value"` 写法，与 config.lua 的约定一致
    lua_value() {
        sed -n "s/^[[:space:]]*$1[[:space:]]*=[[:space:]]*\"\([^\"]*\)\".*/\1/p" "$CONFIG" | head -1
    }
    TUM="$(lua_value outputTum)"
    PLY="$(lua_value plyPath)"
    if [ -z "$TUM" ]; then
        echo "⚠️  配置里 dataset.replay.outputTum 为空，没有可绘制的轨迹" >&2
        exit 0
    fi
    if [ ! -f "$TUM" ]; then
        echo "⚠️  轨迹文件不存在: $TUM（回放是否未完成初始化？）" >&2
        exit 0
    fi
    echo "==> 绘制轨迹图"
    plot_args=("$TUM" --save "${TUM%.tum}.png")
    [ -n "$PLY" ] && plot_args+=(--ply "$PLY")
    python3 "$ROOT/scripts/plot_tum.py" "${plot_args[@]}"
fi
