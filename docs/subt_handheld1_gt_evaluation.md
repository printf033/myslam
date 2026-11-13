# SUBT Handheld1 真值情况与绝对位置评估

日期：2026-11-12。数据：`data/subt_tunnel_gt/`（来自
`github.com/subtchallenge/systems_tunnel_ground_truth`，master）。

## 结论速览

1. **Handheld1 没有逐时间戳的 GT 轨迹**。SUBT 隧道数据集只提供
   artifact/fiducial 点真值（30 个 EX artifact + fiducial 表，含 EX→UTM 变换）
   和全场景点云扫描，不提供逐帧位姿。
2. 用刚体配准把 VIO 轨迹对齐到隧道点云后，**轨迹与巷道几何吻合良好**：
   点到点云最近邻距离 mean 0.547 m / median 0.416 m / p90 1.052 m / max 3.64 m。
3. **该序列是西侧斜巷段的折返路线，全程未经过任何 artifact**（最近 artifact
   距轨迹 70–390 m），因此 artifact 表无法用于 Handheld1 的逐点绝对定位评估。
4. 配准后 Z 向变化仅 5.6 m（局部帧里看到的 21 m "Z 漂移"主要是全局倾斜，
   已被旋转分量吸收），与隧道实际高差相符，**不构成 VIO 尺度/重力漂移证据**。

## 数据源

| 文件 | 内容 |
|---|---|
| `Tunnel_Artifact_Ground_Truth.xlsx` | 30 个 artifact 的 EX / UTM 坐标（A/B 两列，位置几乎相同） |
| `Tunnel_Fiducials_Ground_Truth.xlsx` | fiducial 表 + EX→UTM(17N) 变换矩阵（approximate） |
| `tunnel_scan.pcd` | 全场景 ASCII 点云，2 143 410 点，EX 帧 |
| `readme.md` | 坐标系说明：`ex` / `sr` / `utm`（UTM 17N） |

高分辨率扫描（FullRes，约 20 GB）在 S3，未拉取。

## 配准方法与结果

- 方法：VIO 轨迹（`/tmp/baseline_subt_full.tum`，93.9–808.2 s，3430 点）
  对点云做刚体变换（6 组初值、ICP 精化），取 cost 最优。
- 最优解：cost 1875.7；rotvec [0.246, −0.164, 2.481]，tvec [−0.71, −21.36, 1.90]。
- 残差（轨迹点→点云最近邻）：mean 0.547 m / median 0.416 m / p90 1.052 m /
  max 3.64 m；按时间分 10 箱无单调漂移趋势 → **刚体变换足够，无需尺度因子**。
- 复跑：`python3 /tmp/reg.py`（约 1–2 min），结果在 `/tmp/reg_result.json`、
  `/tmp/trj_ex.npy`；俯视图 `/tmp/subt_topview.png`。

## H1/H2 判别（为什么配准不是局部最优错解）

- **H1**：人沿东西主巷单程 ~393 m，VIO 把直线扭曲成锯齿、配准落进西侧斜巷。
- **H2**：人在西侧斜巷段内反复折返（总路径 393 m），配准正确，只是没经过 artifact。

判别证据（支持 H2）：

1. **折返比 24×**：配准后轨迹路径长 393.2 m，净位移仅 16.3 m
   （起点 EX [−0.58, −21.54, 2.87]，终点 [−3.97, −5.56, 3.02]）。
   单程直线不可能出现这种形态。
2. **视频帧**（`cam_0/`，约 24.5 Hz，19839 帧）：起点 frame 2300 与终点
   frame 19600（红外模式）都是同一人坐在木凳上、旁边同一木楼梯 → 起终点为
   同一位置，是回路/折返路线；frame 17150 的笔直长栈道与斜巷几何一致。
3. **残差形态**：配准残差中位仅 0.4 m 且末段最好（0.049 m），若是把 393 m
   直线硬压进 40 m 斜巷，不可能出现这种量级的逐点贴合。

## 对评估的含义

- **可用的绝对精度指标**：配准后轨迹 vs 巷道几何（median ≈ 0.4 m、p90 ≈ 1.05 m），
  该数值包含刚体配准自身误差，应理解为"轨迹与真实巷道的贴合程度"而非逐点定位误差。
- **不可用的指标**：artifact 逐点定位评估——Handheld1 全程未进入任何 artifact
  所在巷道段（30 个 artifact 全部位于东西主巷 + 北分支网，距轨迹 70–390 m）。
- **裸 IMU 积分不可用**（已验证）：加速度计未去偏，速度发散至 ~27 m/s、航向锁死，
  总位移 13.6 km；不要用此路做交叉验证。

## 局限与后续建议

- Handheld1 对应 EX course 的 A/B config 未最终确认（两列 artifact 位置几乎
  相同，仅 #4/#6 有差异，影响可忽略）。
- 若需要逐点绝对定位评估，应选用**经过 artifact 巷道的序列**（如东西主巷单程
  的序列），或改用 fiducial/ARUCO 可见帧做局部匹配。
- 全场景 FullRes 扫描（S3，20 GB）可提供更高密度的几何基准，当前 2.1 M 点
  已足够本评估。
