#!/usr/bin/env python3
"""Generate the architecture diagrams (docs/architecture.svg, docs/architecture_zh.svg).

Layout is shared between languages; only the strings differ. Coordinates are
computed here rather than hand-written so alignment stays exact.

The two columns are the two sample rates. Each column is split into an
initialization stage and a tracking stage, because within a column the two
stages are mutually exclusive in code (`VioSystem::run()` branches on the
current `SystemState`), not sequential.
"""

from __future__ import annotations

import pathlib

W, H = 1040, 780

# --- palette ---------------------------------------------------------------
INK = "#0f172a"          # primary text
MUTED = "#64748b"        # secondary text
ARROW = "#64748b"        # connector strokes
PANEL_FILL = "#f8fafc"   # outer group container
PANEL_LINE = "#cbd5e1"
STAGE_FILL = "#ffffff"   # per-stage container
STAGE_LINE = "#e2e8f0"
BOX_FILL = "#ffffff"
BOX_LINE = "#475569"
INPUT_FILL = "#f1f5f9"
INPUT_LINE = "#64748b"
EMPH_FILL = "#eff6ff"    # VioBackend, the shared component
EMPH_LINE = "#2563eb"

SANS = ("'Noto Sans CJK SC','Source Han Sans SC','PingFang SC',"
        "'Segoe UI',Roboto,Helvetica,Arial,sans-serif")
MONO = "'JetBrains Mono','SFMono-Regular',Consolas,'Liberation Mono',monospace"

# --- geometry --------------------------------------------------------------
IN_Y, IN_H, IN_W = 44, 78, 300
IN_AX, IN_BX = 110, 630
CX_L, CX_R, CX_MID = IN_AX + IN_W // 2, IN_BX + IN_W // 2, W // 2

MERGE_Y = 152
PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 60, 186, 920, 442

COL_W = 300
BOX_H = 68
STAGE_PAD = 10
STAGE_TITLE_H = 26

STAGE1_Y = 286
STAGE1_H = STAGE_TITLE_H + BOX_H + STAGE_PAD      # 104
STAGE2_Y = STAGE1_Y + STAGE1_H + 16              # 406
STAGE2_H = STAGE_TITLE_H + BOX_H + 20 + BOX_H + STAGE_PAD   # 192

BOX1_Y = STAGE1_Y + STAGE_TITLE_H                # single box in stage 1
BOX2A_Y = STAGE2_Y + STAGE_TITLE_H
BOX2B_Y = BOX2A_Y + BOX_H + 20

OUT_W, OUT_H, OUT_Y = 280, 72, 668

TEXT = {
    "en": {
        "sim": ("VioSimulator", "synthetic IMU / camera"),
        "seq": ("SubtSequence", "recorded sequence reader"),
        "loop": "VioSystem::run()",
        "loop_sub": "dual-rate main loop",
        "imu": "IMU 200 Hz",
        "cam": "Camera 20 Hz",
        "stage_init": "initialization",
        "stage_track": "tracking",
        "l1": ("VioInitializer", "feedImu"),
        "l2": ("VioBackend", "integrateImu"),
        "l3": ("HighRatePropagator", "propagate"),
        "r1": ("VioInitializer", "tryGravityAlignment"),
        "r2": ("KeyframeSelector", "evaluate"),
        "r3": ("VioBackend", "processKeyframe"),
        "reset": "reset base after optimization",
        "keyframe": "isKeyframe",
        "out": ("RunReport", "RMSE / max error"),
        "sensors": "sensor input",
    },
    "zh": {
        "sim": ("VioSimulator", "合成 IMU / 相机观测"),
        "seq": ("SubtSequence", "序列读取"),
        "loop": "VioSystem::run()",
        "loop_sub": "双速率主循环",
        "imu": "IMU 200 Hz",
        "cam": "Camera 20 Hz",
        "stage_init": "初始化阶段",
        "stage_track": "追踪阶段",
        "l1": ("VioInitializer", "feedImu"),
        "l2": ("VioBackend", "integrateImu"),
        "l3": ("HighRatePropagator", "propagate"),
        "r1": ("VioInitializer", "tryGravityAlignment"),
        "r2": ("KeyframeSelector", "evaluate"),
        "r3": ("VioBackend", "processKeyframe"),
        "reset": "优化后重置外推基线",
        "keyframe": "isKeyframe",
        "out": ("RunReport", "RMSE / 最大偏差"),
        "sensors": "传感器输入",
    },
}


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def box(x, y, w, h, fill, line, sw=1.5, rx=8) -> str:
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
            f'fill="{fill}" stroke="{line}" stroke-width="{sw}"/>')


def component(x, y, w, h, title, method, emph=False) -> str:
    fill = EMPH_FILL if emph else BOX_FILL
    line = EMPH_LINE if emph else BOX_LINE
    cx = x + w / 2
    parts = [box(x, y, w, h, fill, line, sw=1.8 if emph else 1.5, rx=8)]
    parts.append(
        f'<text x="{cx}" y="{y + h / 2 - 5}" text-anchor="middle" font-family="{SANS}" '
        f'font-size="15" font-weight="600" fill="{INK}">{esc(title)}</text>')
    parts.append(
        f'<text x="{cx}" y="{y + h / 2 + 17}" text-anchor="middle" font-family="{MONO}" '
        f'font-size="12.5" fill="{MUTED}">{esc(method)}</text>')
    return "\n".join(parts)


def stage(x, y, w, h, title) -> str:
    """A per-rate stage container with its title inside the top edge."""
    return "\n".join([
        box(x, y, w, h, STAGE_FILL, STAGE_LINE, sw=1.2, rx=10),
        f'<text x="{x + w / 2}" y="{y + 18}" text-anchor="middle" font-family="{SANS}" '
        f'font-size="11.5" fill="{MUTED}">{esc(title)}</text>',
    ])


def arrow(x1, y1, x2, y2, dashed=False) -> str:
    dash = ' stroke-dasharray="5 4"' if dashed else ""
    return (f'<path d="M {x1} {y1} L {x2} {y2}" fill="none" stroke="{ARROW}" '
            f'stroke-width="1.6" marker-end="url(#head)"{dash}/>')


def line(x1, y1, x2, y2) -> str:
    return (f'<path d="M {x1} {y1} L {x2} {y2}" fill="none" stroke="{ARROW}" '
            f'stroke-width="1.6"/>')


def label(x, y, text, size=11, fill=MUTED, mono=False, weight="400", anchor="middle") -> str:
    fam = MONO if mono else SANS
    return (f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-family="{fam}" '
            f'font-size="{size}" font-weight="{weight}" fill="{fill}">{esc(text)}</text>')


def build(lang: str) -> str:
    t = TEXT[lang]
    stage_w = COL_W + 2 * STAGE_PAD
    panel_bottom = PANEL_Y + PANEL_H
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}" role="img" aria-label="myslam architecture">',
        '<defs>',
        '<marker id="head" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
        f'markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M 0 1 L 9 5 L 0 9 z" fill="{ARROW}"/></marker>',
        '</defs>',
        f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
    ]

    # --- sensor inputs ------------------------------------------------------
    out.append(label(CX_MID, 30, t["sensors"], size=12, weight="600"))
    for x, key in ((IN_AX, "sim"), (IN_BX, "seq")):
        title, sub = t[key]
        cx = x + IN_W / 2
        out.append(box(x, IN_Y, IN_W, IN_H, INPUT_FILL, INPUT_LINE, rx=8))
        out.append(label(cx, IN_Y + 34, title, size=15, fill=INK, mono=True, weight="600"))
        out.append(label(cx, IN_Y + 56, sub, size=12))

    out.append(line(CX_L, IN_Y + IN_H, CX_L, MERGE_Y))
    out.append(line(CX_R, IN_Y + IN_H, CX_R, MERGE_Y))
    out.append(line(CX_L, MERGE_Y, CX_R, MERGE_Y))
    out.append(arrow(CX_MID, MERGE_Y, CX_MID, PANEL_Y))

    # --- main loop group ----------------------------------------------------
    out.append(f'<rect x="{PANEL_X}" y="{PANEL_Y}" width="{PANEL_W}" height="{PANEL_H}" '
               f'rx="12" fill="{PANEL_FILL}" stroke="{PANEL_LINE}" stroke-width="1.6" '
               f'stroke-dasharray="7 5"/>')
    out.append(label(CX_MID, PANEL_Y + 34, t["loop"], size=16, fill=INK, mono=True, weight="600"))
    out.append(label(CX_MID, PANEL_Y + 56, t["loop_sub"], size=12))

    out.append(label(CX_L, STAGE1_Y - 16, t["imu"], size=12, fill=INK, mono=True, weight="600"))
    out.append(label(CX_R, STAGE1_Y - 16, t["cam"], size=12, fill=INK, mono=True, weight="600"))

    # --- two rate columns, each split by lifecycle stage --------------------
    for col_x, tag in ((IN_AX, "l"), (IN_BX, "r")):
        sx = col_x - STAGE_PAD
        out.append(stage(sx, STAGE1_Y, stage_w, STAGE1_H, t["stage_init"]))
        out.append(stage(sx, STAGE2_Y, stage_w, STAGE2_H, t["stage_track"]))
        out.append(component(col_x, BOX1_Y, COL_W, BOX_H, *t[f"{tag}1"]))
        out.append(component(col_x, BOX2A_Y, COL_W, BOX_H, *t[f"{tag}2"], emph=(tag == "l")))
        out.append(component(col_x, BOX2B_Y, COL_W, BOX_H, *t[f"{tag}3"], emph=(tag == "r")))

    # connectors inside the tracking stage
    out.append(arrow(CX_L, BOX2A_Y + BOX_H, CX_L, BOX2B_Y - 2))
    out.append(arrow(CX_R, BOX2A_Y + BOX_H, CX_R, BOX2B_Y - 2))
    out.append(label(CX_R + 12, BOX2A_Y + BOX_H + 12, t["keyframe"], size=10.5, anchor="start"))

    # processKeyframe feeds the optimized state back into the propagator
    reset_y = BOX2B_Y + BOX_H // 2
    out.append(arrow(IN_BX - 2, reset_y, IN_AX + COL_W + 2, reset_y))
    out.append(label(CX_MID, reset_y - 11, t["reset"], size=10.5))

    # --- accuracy report ----------------------------------------------------
    joiner_y = STAGE2_Y + STAGE2_H + 16
    out.append(line(CX_R, BOX2B_Y + BOX_H, CX_R, joiner_y))
    out.append(line(CX_R, joiner_y, CX_MID, joiner_y))
    out.append(arrow(CX_MID, joiner_y, CX_MID, OUT_Y))

    x = CX_MID - OUT_W / 2
    out.append(box(x, OUT_Y, OUT_W, OUT_H, BOX_FILL, BOX_LINE, rx=8))
    out.append(label(CX_MID, OUT_Y + 32, t["out"][0], size=15, fill=INK, mono=True, weight="600"))
    out.append(label(CX_MID, OUT_Y + 54, t["out"][1], size=12))

    out.append('</svg>')
    return "\n".join(out) + "\n"


def main() -> None:
    root = pathlib.Path(__file__).resolve().parent.parent
    docs = root / "docs"
    docs.mkdir(exist_ok=True)
    for lang, name in (("en", "architecture.svg"), ("zh", "architecture_zh.svg")):
        target = docs / name
        target.write_text(build(lang), encoding="utf-8")
        print(f"wrote {target.relative_to(root)} ({target.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
