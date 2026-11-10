#!/usr/bin/env python3
"""静态检查 src/ 的分层依赖方向。

CMake 的 INTERFACE target 能表达"这一层链接了哪一层"，但 header-only 项目里
所有头都挂在同一个 include 根下，`#include "backend/..."` 从任何 target 都能
解析成功——构建系统本身拦不住越层引用。本脚本补上那一半机制：

    读取每个 src/<layer>/*.hpp 的 #include "..."，
    取其第一段作为目标层，
    若目标层不在该层允许的集合内，报违规并以非零码退出。

规则与 src/CMakeLists.txt 中 target_link_libraries 的依赖图保持一致；修改
分层时两处都要改，否则 check-layers 会失败——这正是想要的提醒。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

# 每层允许包含的层（含自身）
LAYER_RULES: dict[str, set[str]] = {
    # 契约层：不依赖任何实现层（也不允许反向依赖适配器）
    "domain": {"domain"},
    # 非算法数据与配置解析
    "common": {"common", "domain"},
    "config": {"config", "common", "domain"},
    "sensor": {"sensor", "common", "domain"},
    "frontend": {"frontend", "domain"},
    # 适配器：领域 + 第三方库。领域层不得反向包含 adapters。
    "adapters": {"adapters", "domain"},
    # 编排核心：只允许领域契约
    "pipeline": {"pipeline", "domain"},
    # 编排层：消费流水线、数据源与适配器注册，不依赖显示层
    "app": {"app", "common", "domain", "sensor", "frontend", "pipeline", "adapters"},
    # 显示层在最上层：实现 app/run_observer.hpp 的接口
    "viz": {"viz", "app", "common", "domain"},
}

INCLUDE_RE = re.compile(r'^\s*#include\s+"([^"]+)"')


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"用法: {argv[0]} <项目根目录>", file=sys.stderr)
        return 2

    src = Path(argv[1]) / "src"
    if not src.is_dir():
        print(f"找不到源目录: {src}", file=sys.stderr)
        return 2

    violations: list[str] = []
    checked = 0

    for path in sorted(src.rglob("*.hpp")):
        relative = path.relative_to(src)
        layer = relative.parts[0]
        if layer not in LAYER_RULES:
            continue  # src 根下的聚合头（vio.hpp 等）不参与分层检查

        allowed = LAYER_RULES[layer]
        text = path.read_text(encoding="utf-8")
        for lineno, line in enumerate(text.splitlines(), start=1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            include = match.group(1)
            if "/" not in include:
                continue  # 同目录/第三方/生成头，不属于分层范围
            dependency = include.split("/", 1)[0]
            if dependency not in LAYER_RULES:
                continue
            checked += 1
            if dependency not in allowed:
                violations.append(
                    f"{path}:{lineno}: {layer}/ 不允许包含 {include}"
                    f"（允许: {', '.join(sorted(allowed))}）"
                )

    if violations:
        print(f"分层检查失败，共 {len(violations)} 处越层引用（已检查 {checked} 条跨目录 include）：",
              file=sys.stderr)
        for violation in violations:
            print(f"  {violation}", file=sys.stderr)
        return 1

    print(f"分层检查通过：{checked} 条跨目录 include 均符合依赖方向。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
