#pragma once

/**
 * @file visual_initializer.hpp
 * @brief 运动起始（动态）初始化：视觉几何 + IMU 预积分闭式对齐。
 *
 * @par 解决什么问题
 *
 * 静止初始化要求设备先静下来：判方差、对齐重力、把陀螺均值当零偏。从
 * 运动状态开始的序列（手持、车载、机器人已在上电前移动）会永远卡在
 * 静止检测上，现有实现只能靠「超时后放宽为非静止对齐」兜底——而那时
 * 初始速度被置零，估计器必须自己把它拉回来，起步阶段因此有明显损失。
 *
 * 本类用一段窗口内的视觉观测与 IMU 数据直接解出：尺度、初始速度、重力
 * 方向偏离量。核心是一条线性方程（每个非首帧给出 3 个方程）：
 *
 * @f[
 *   s \cdot \mathbf{p}_{0j}^{\text{vis}}
 *     = \Delta\mathbf{p}_{0j}^{\text{imu}}
 *     + \mathbf{v}_0 \Delta t
 *     + \tfrac{1}{2}\delta\mathbf{g}\Delta t^{2}
 * @f]
 *
 * 其中 @f$ s @f$ 是「视觉基线 = 1」到真实尺度的换算，@f$ \Delta\mathbf{p} @f$
 * 是从首帧起、以「速度为零」为假设做 IMU 传播得到的位移，@f$ \delta\mathbf{g} @f$
 * 是真实重力相对配置重力（@c (0,0,-g)）的偏差。未知量 7 个，因此至少需要
 * 4 帧（给出 9 个方程）。
 *
 * @par 尺度一致性的来源
 *
 * 单目两视图只能恢复到「尺度未知」的位移。若每对帧各自做本质矩阵分解，
 * 各对的尺度互不相同，线性系统无法成立。做法是：**先把首末帧三角化，
 * 再用同一批 3D 点对所有中间帧做 PnP**——这样所有位姿都在同一尺度下。
 * 这也是本实现没有直接拿「逐对分解结果」拼轨迹的原因。
 *
 * @par 重力模长为什么是硬约束
 *
 * 未知量是尺度、初始速度与**重力方向**，重力模长由 @c |g| = 9.81 精确给定，
 * 因此切平面上只剩两个自由度。解 7 个未知量再事后检查模长的做法允许解出的
 * 重力偏离 50%，激励不足时尺度与重力会互相代偿，事后检查只能发现、不能纠正。
 * 迭代按残差单调下降接受，单步转动超过 30° 即停：短窗口下重力方向几乎不可观，
 * 无保护的迭代会正反馈发散。
 *
 * @par 坐标系
 *
 * IMU 传播从单位姿态起算、配置重力取 @c (0,0,-G)，因此位移活在**首帧 body 系**；
 * 视觉位姿活在**首帧相机系**。两者差一个外参旋转，必须先转到同一系再进入线性
 * 系统，否则 90° 的坐标系差会被当成重力偏差吸收掉，且不报任何错。
 *
 * @par 已知限制
 *
 * - 只做线性解，没有非线性精化（Ceres 精化是后续工作）。
 * - 陀螺零偏由「视觉相对旋转 vs IMU 相对旋转」最小二乘给出；短窗口下视觉
 *   旋转噪声大时该估计会被物理上限（0.2 rad/s）拒绝，退回零偏为零。
 * - 加速度计零偏不可观（与重力方向耦合），仍由后端在线估计。
 * - 初始速度与重力的可观性依赖足够的旋转激励；尺度与重力在小窗口内接近
 *   退化时，两者都可能给出数量级偏差——测试里对此不作精度断言，只钉机制。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/propagator.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <span>
#include <unordered_map>
#include <vector>

namespace myslam::vio
{

    /**
     * @brief 视觉-惯性动态初始化器（实现 @c domain::IDynamicInitializer）
     */
    class VisualImuInitializer final : public domain::IDynamicInitializer
    {
    public:
        VisualImuInitializer(const domain::InitializationConfig &initialization, const domain::ImuConfig &imu,
                             const camera::CameraIntrinsics &intrinsics,
                             const Eigen::Quaterniond &bodyFromCamera = Eigen::Quaterniond::Identity())
            : initialization_(initialization),
              // 顺序必须与成员声明一致（initialization_ → bodyFromCamera_ → imu_），
              // 否则每个包含本头的 TU 都会报 -Wreorder。
              bodyFromCamera_(bodyFromCamera.normalized()),
              imu_(imu),
              K_((cv::Mat_<double>(3, 3) << intrinsics.pinholeFx(), 0.0, intrinsics.pinholeCx(), 0.0,
                  intrinsics.pinholeFy(), intrinsics.pinholeCy(), 0.0, 0.0, 1.0)),
              focal_(0.5 * (intrinsics.pinholeFx() + intrinsics.pinholeFy()))
        {
        }

        void reset() override {}

        [[nodiscard]] domain::DynamicInitResult tryInitialize(std::span<const domain::Keyframe> frames,
                                                              std::span<const domain::ImuSlice> imu) override
        {
            domain::DynamicInitResult result;

            // 7 个未知量需要至少 3 个非首帧（9 个方程）。
            if (frames.size() < 4)
            {
                result.detail = std::format("帧数 {} 不足 4", frames.size());
                return result;
            }

            const domain::Keyframe &first = frames.front();
            const domain::Keyframe &last = frames.back();

            // ---------------------------------------------------------------
            // 1. 首末帧共视特征
            // ---------------------------------------------------------------
            std::unordered_map<domain::FeatureId, domain::Vec2> firstUv;
            firstUv.reserve(first.observations.size());
            for (const domain::Observation &observation : first.observations)
            {
                firstUv[observation.id] = observation.uv;
            }

            std::vector<cv::Point2f> pointsFirst;
            std::vector<cv::Point2f> pointsLast;
            std::vector<domain::FeatureId> matchedIds;
            for (const domain::Observation &observation : last.observations)
            {
                const auto it = firstUv.find(observation.id);
                if (it == firstUv.end())
                {
                    continue;
                }
                pointsFirst.emplace_back(static_cast<float>(it->second.x()), static_cast<float>(it->second.y()));
                pointsLast.emplace_back(static_cast<float>(observation.uv.x()), static_cast<float>(observation.uv.y()));
                matchedIds.push_back(observation.id);
            }
            if (pointsFirst.size() < 12)
            {
                result.detail = std::format("首末帧共视 {} 点，不足 12", pointsFirst.size());
                return result;
            }

            // ---------------------------------------------------------------
            // 2. 本质矩阵 -> 相对位姿
            // ---------------------------------------------------------------
            // 阈值取 2 像素对应的归一化尺度：光流与像素噪声都在 1 px 量级，
            // 1 px 的阈值会把噪声本身判成外点，内点率随噪声水平剧烈抖动。
            const double ransacThreshold = 2.0 / std::max(1.0, focal_);
            cv::Mat inlierMask;
            // 点序必须与 recoverPose、三角化三处一致：都用 (首帧, 末帧)。
            // 混用会让三角形的点在末帧系，而后面 PnP 又按首帧系解释位姿，
            // 两者相差一个未取逆的外参——症状是几乎全部点落在相机后方。
            const cv::Mat essential =
                cv::findEssentialMat(pointsFirst, pointsLast, K_, cv::RANSAC, ransacThreshold, 0.999, inlierMask);
            if (essential.empty())
            {
                result.detail = "本质矩阵求解失败（退化运动或匹配错误）";
                return result;
            }

            cv::Mat rotationFirstToLast;
            cv::Mat translationDirection;
            // 返回值满足 x_last = R * x_first + t，因此 P_last = K[R|t] 直接可用。
            const int inliers = cv::recoverPose(essential, pointsFirst, pointsLast, K_, rotationFirstToLast,
                                                translationDirection, inlierMask);
            // 门限取 8：本质矩阵最少需要 5 对点，8 对留出三倍余量。更严的门限
            // 会在低视差（缓慢平移）场景下把本来可用的解挡在门外。
            if (inliers < 8 || cv::norm(translationDirection) < 1e-9)
            {
                result.detail = std::format("位姿恢复失败：内点 {}/{}，平移模长 {:.2e}", inliers, pointsFirst.size(),
                                            cv::norm(translationDirection));
                return result;
            }

            // ---------------------------------------------------------------
            // 3. 三角化（首帧相机系，基线尺度归一）
            // ---------------------------------------------------------------
            // recoverPose 直接给出 3x3 旋转矩阵，不需要（也不能）再过一次
            // Rodrigues：把矩阵当旋转向量转过去会得到一个 3x1 向量，再被当作
            // 旋转使用时，三角化出来的深度会随机变号——症状是「内点不少但
            // 深度为正的点很少」，容易被误判成基线太小。
            if (rotationFirstToLast.rows != 3 || rotationFirstToLast.cols != 3)
            {
                result.detail = std::format("recoverPose 返回的旋转尺寸异常: {}x{}", rotationFirstToLast.rows,
                                            rotationFirstToLast.cols);
                return result;
            }
            if (translationDirection.rows != 3)
            {
                translationDirection = translationDirection.t();
            }

            // 逐元素写入而不是对子矩阵 copyTo：`Rect` 视图是非连续存储，
            // copyTo 到固定尺寸子块会触发 OpenCV 的 create 断言。
            cv::Mat poseLast = cv::Mat::eye(3, 4, CV_64F);
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    poseLast.at<double>(row, column) = rotationFirstToLast.at<double>(row, column);
                }
                poseLast.at<double>(row, 3) = translationDirection.at<double>(row);
            }

            // 在归一化平面上三角化：投影矩阵就是 [I|0] 与 [R|t]，像素内参
            // 不再参与。这样若三角化结果仍然错，只可能是位姿的问题，排查
            // 范围从「内参 × 位姿 × 点序」缩到「位姿」一项。
            std::vector<cv::Point2f> normalizedFirst;
            std::vector<cv::Point2f> normalizedLast;
            cv::undistortPoints(pointsFirst, normalizedFirst, K_, cv::noArray());
            cv::undistortPoints(pointsLast, normalizedLast, K_, cv::noArray());

            const cv::Mat projectionFirst = cv::Mat::eye(3, 4, CV_64F);
            const cv::Mat projectionLast = poseLast;

            cv::Mat homogeneous;
            cv::triangulatePoints(projectionFirst, projectionLast, normalizedFirst, normalizedLast, homogeneous);

            std::vector<cv::Point3f> points3D;
            std::vector<domain::FeatureId> triangulatedIds;
            int maskedCount = 0;
            int positiveDepthCount = 0;
            for (int i = 0; i < homogeneous.cols; ++i)
            {
                if (inlierMask.at<uchar>(i) == 0)
                {
                    continue;
                }
                ++maskedCount;
                const float w = homogeneous.at<float>(3, i);
                if (std::abs(w) < 1e-9F)
                {
                    continue;
                }
                const cv::Point3f point(homogeneous.at<float>(0, i) / w, homogeneous.at<float>(1, i) / w,
                                        homogeneous.at<float>(2, i) / w);
                // 深度必须为正且有限：落在首帧相机后方的点会让后续 PnP 直接错
                if (!(point.z > 1e-3F) || !std::isfinite(point.z) || point.z > 1e4F)
                {
                    continue;
                }
                ++positiveDepthCount;
                points3D.push_back(point);
                triangulatedIds.push_back(matchedIds[static_cast<std::size_t>(i)]);
            }
            if (points3D.size() < 8)
            {
                result.detail = std::format("三角化有效点 {} 个，不足 8（内点掩码 {} 个，深度为正 {} 个）",
                                            points3D.size(), maskedCount, positiveDepthCount);
                return result;
            }

            // ---------------------------------------------------------------
            // 4. 逐帧 PnP：把中间帧解到同一尺度
            // ---------------------------------------------------------------
            std::vector<domain::Pose> poses;
            std::vector<double> stamps;
            poses.reserve(frames.size());
            stamps.reserve(frames.size());
            poses.push_back(domain::Pose{});
            stamps.push_back(first.timestamp);

            for (std::size_t frameIndex = 1; frameIndex < frames.size(); ++frameIndex)
            {
                std::unordered_map<domain::FeatureId, domain::Vec2> uvMap;
                uvMap.reserve(frames[frameIndex].observations.size());
                for (const domain::Observation &observation : frames[frameIndex].observations)
                {
                    uvMap[observation.id] = observation.uv;
                }

                std::vector<cv::Point3f> objectPoints;
                std::vector<cv::Point2f> imagePoints;
                for (std::size_t k = 0; k < triangulatedIds.size(); ++k)
                {
                    const auto it = uvMap.find(triangulatedIds[k]);
                    if (it == uvMap.end())
                    {
                        continue;
                    }
                    objectPoints.push_back(points3D[k]);
                    imagePoints.emplace_back(static_cast<float>(it->second.x()), static_cast<float>(it->second.y()));
                }
                // PnP 最少 4 对，留一倍余量避免病态
                if (objectPoints.size() < 8)
                {
                    result.detail = std::format("第 {} 帧 PnP 对应点 {} 个，不足 8", frameIndex, objectPoints.size());
                    return result;
                }

                cv::Mat frameRotationVector;
                cv::Mat frameTranslation;
                if (!cv::solvePnP(objectPoints, imagePoints, K_, cv::noArray(), frameRotationVector, frameTranslation,
                                  false, cv::SOLVEPNP_EPNP))
                {
                    result.detail = std::format("第 {} 帧 PnP 失败", frameIndex);
                    return result;
                }
                cv::solvePnPRefineLM(objectPoints, imagePoints, K_, cv::noArray(), frameRotationVector,
                                     frameTranslation);

                cv::Mat frameRotation;
                cv::Rodrigues(frameRotationVector, frameRotation);
                Eigen::Matrix3d rotationCameraFromWorld;
                cv::cv2eigen(frameRotation, rotationCameraFromWorld);

                // solvePnP 求的是「世界 -> 相机」的变换，而 poses 需要的是
                // 「世界 <- 相机」（相机在首帧系中的位姿）。直接用会得到相反
                // 的位移方向，线性系统因此解出负尺度——这个错误不会报错，
                // 只会让整条轨迹跑在镜像世界里。
                const Eigen::Matrix3d rotationWorldFromCamera = rotationCameraFromWorld.transpose();
                const Eigen::Vector3d translationCameraFromWorld(
                    frameTranslation.at<double>(0), frameTranslation.at<double>(1), frameTranslation.at<double>(2));
                const Eigen::Vector3d translationWorldFromCamera =
                    -(rotationWorldFromCamera * translationCameraFromWorld);
                poses.push_back(domain::Pose{Eigen::Quaterniond(rotationWorldFromCamera),
                                             domain::Vec3(translationWorldFromCamera)});
                stamps.push_back(frames[frameIndex].timestamp);
            }

            // ---------------------------------------------------------------
            // 5. 旋转激励检查：没有转动就没有可观性
            // ---------------------------------------------------------------
            const Eigen::Matrix3d rotationSpan = poses.back().orientation.toRotationMatrix();
            const double spannedDegrees = Eigen::AngleAxisd(rotationSpan).angle() * 180.0 / std::numbers::pi;
            if (spannedDegrees < initialization_.dynamicMinRotationDeg)
            {
                result.detail = std::format("窗口内累计旋转 {:.1f}°，低于门限 {:.1f}°", spannedDegrees,
                                            initialization_.dynamicMinRotationDeg);
                return result;
            }

            // ---------------------------------------------------------------
            // 6. IMU 传播（写成可重跑的：估出零偏后要用它再积一次）
            // ---------------------------------------------------------------
            const auto expMap = [](const Eigen::Vector3d &rotationVector) -> Eigen::Matrix3d
            {
                const double angle = rotationVector.norm();
                if (angle < 1e-12)
                {
                    return Eigen::Matrix3d::Identity();
                }
                return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
            };
            const auto logMap = [](const Eigen::Matrix3d &rotation)
            {
                const Eigen::AngleAxisd angleAxis(rotation);
                return Eigen::Vector3d(angleAxis.angle() * angleAxis.axis());
            };

            const auto propagateImu = [&](const domain::Bias &bias, std::vector<domain::Vec3> &displacement,
                                          std::vector<domain::Vec3> &velocity,
                                          std::vector<Eigen::Quaterniond> &orientations) -> bool
            {
                displacement.assign(frames.size(), domain::Vec3::Zero());
                velocity.assign(frames.size(), domain::Vec3::Zero());
                orientations.assign(frames.size(), Eigen::Quaterniond::Identity());

                domain::KinematicState baseline;
                baseline.timestamp = first.timestamp;
                baseline.pose = domain::Pose{};
                baseline.velocity = domain::Vec3::Zero();
                baseline.bias = bias;

                domain::StatePropagator propagator(imu_);
                propagator.reset(baseline);

                std::size_t nextFrame = 1;
                for (const domain::ImuSlice &slice : imu)
                {
                    const domain::KinematicState state = propagator.propagate(slice);
                    // 判据用「切片覆盖到的时刻」= timestamp + dt，而不是样本起点。
                    // 用起点会让最后一帧永远覆盖不到。
                    const double coveredUntil = slice.sample.timestamp + slice.dt;
                    while (nextFrame < frames.size() && coveredUntil >= frames[nextFrame].timestamp - 1e-9)
                    {
                        displacement[nextFrame] = state.pose.position;
                        velocity[nextFrame] = state.velocity;
                        orientations[nextFrame] = state.pose.orientation;
                        ++nextFrame;
                    }
                    if (nextFrame >= frames.size())
                    {
                        break;
                    }
                }
                return nextFrame >= frames.size();
            };

            std::vector<domain::Vec3> imuDisplacement;
            std::vector<domain::Vec3> imuVelocity;
            std::vector<Eigen::Quaterniond> imuRotation;
            if (!propagateImu(domain::Bias{}, imuDisplacement, imuVelocity, imuRotation))
            {
                result.detail = std::format("IMU 未覆盖到末帧（共 {} 帧）", frames.size());
                return result;
            }

            // ---------------------------------------------------------------
            // 6.5 陀螺零偏：视觉相对旋转 - IMU 相对旋转 的最小二乘
            // ---------------------------------------------------------------
            //
            // 零偏未估时它是被"吸收"进其他状态的：线性系统会用一个偏掉的
            // 重力方向去补偿陀螺漂移，尺度与速度跟着一起歪。旋转通道是唯一
            // 与平移解耦的观测，用它单独解 bg 代价很低。
            //
            // 模型：积分用的是 ω_meas，真值是 ω_meas - bg，因此一段区间内的
            // 相对旋转满足 dR(0) = dR(bg)·Exp(bg·dt)，即 dR(bg) = dR(0)·Exp(-bg·dt)。
            domain::Bias estimatedBias{};
            {
                const auto residuals = [&](const Eigen::Vector3d &bg, std::vector<Eigen::Vector3d> &out)
                {
                    out.clear();
                    out.reserve(frames.size());
                    for (std::size_t frameIndex = 1; frameIndex < frames.size(); ++frameIndex)
                    {
                        const double dt = stamps[frameIndex] - stamps[frameIndex - 1];
                        if (!(dt > 1e-6))
                        {
                            continue;
                        }
                        const Eigen::Matrix3d imuRelative = imuRotation[frameIndex - 1].toRotationMatrix().transpose() *
                                                            imuRotation[frameIndex].toRotationMatrix();
                        const Eigen::Matrix3d corrected = imuRelative * expMap(-bg * dt);
                        const Eigen::Matrix3d visualRelativeCamera =
                            poses[frameIndex - 1].orientation.toRotationMatrix().transpose() *
                            poses[frameIndex].orientation.toRotationMatrix();
                        // IMU 积分给出的是 body 系相对旋转，视觉给的是相机系：
                        // 两者差一个外参共轭 R_bc·dR·R_bcᵀ。不转过来，残差里
                        // 就混进一个常量旋转，解出的零偏完全是噪声。
                        const Eigen::Matrix3d bodyFromCameraRotation = bodyFromCamera_.toRotationMatrix();
                        const Eigen::Matrix3d visualRelative =
                            bodyFromCameraRotation * visualRelativeCamera * bodyFromCameraRotation.transpose();
                        out.push_back(logMap(corrected.transpose() * visualRelative));
                    }
                };

                std::vector<Eigen::Vector3d> base;
                residuals(Eigen::Vector3d::Zero(), base);
                if (base.size() >= 3)
                {
                    constexpr double kDelta = 1e-4;
                    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
                    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
                    for (std::size_t k = 0; k < base.size(); ++k)
                    {
                        Eigen::Matrix3d jacobian;
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            Eigen::Vector3d step = Eigen::Vector3d::Zero();
                            step(axis) = kDelta;
                            std::vector<Eigen::Vector3d> plus;
                            std::vector<Eigen::Vector3d> minus;
                            residuals(step, plus);
                            residuals(-step, minus);
                            if (plus.size() != base.size() || minus.size() != base.size())
                            {
                                continue;
                            }
                            jacobian.col(axis) = (plus[k] - minus[k]) / (2.0 * kDelta);
                        }
                        normal += jacobian.transpose() * jacobian;
                        rhs += jacobian.transpose() * base[k];
                    }
                    const Eigen::LDLT<Eigen::Matrix3d> ldlt(normal);
                    if (ldlt.info() == Eigen::Success)
                    {
                        const Eigen::Vector3d bias = ldlt.solve(rhs);
                        // 物理上限：手持 MEMS 的陀螺零偏很少超过 0.2 rad/s。
                        // 超过说明旋转激励不足或视觉旋转有误，退回零偏为零。
                        if (bias.allFinite() && bias.norm() < 0.2)
                        {
                            estimatedBias.gyro = domain::Vec3(bias);
                            mylog::info("[动态初始化] 陀螺零偏估计: [{:.5f}, {:.5f}, {:.5f}] rad/s", bias(0), bias(1),
                                        bias(2));
                        }
                        else
                        {
                            mylog::warn("[动态初始化] 陀螺零偏估计 [{:.4f}, {:.4f}, {:.4f}] rad/s 不合理，保留零初值",
                                        bias(0), bias(1), bias(2));
                        }
                    }
                }
            }

            // 用估出的零偏重新积分，后续线性系统才建立在正确的 IMU 轨迹上
            if (estimatedBias.gyro.norm() > 0.0)
            {
                if (!propagateImu(estimatedBias, imuDisplacement, imuVelocity, imuRotation))
                {
                    result.detail = "带零偏的 IMU 重新积分未覆盖到末帧";
                    return result;
                }
            }

            // ---------------------------------------------------------------
            // 7. 线性最小二乘，|g| 作为硬约束迭代求解
            // ---------------------------------------------------------------
            //
            // 每个非首帧给出 3 个方程：
            //   s·p_vis = Δp_imu + v0·dt + ½·dt²·g
            // 未知量是 s、v0 与**重力方向**。把 g 拆成 |g|·ĝ + w1·t1 + w2·t2
            // （t1/t2 是 ĝ 切平面的正交基），|g| 就由常数精确给定，只剩两个
            // 自由度要解。逐次重解并更新 ĝ，两三步即收敛。
            //
            // 与「解 7 个未知量再事后检查模长」的差别：后者允许解出的 g
            // 模长偏离 50%，激励不足时尺度与重力串在一起互相代偿，事后检查
            // 只能发现、不能纠正。
            const double gravityMagnitude = imu_.gravityMagnitude;
            const double minScale = 1e-4;

            // 坐标系对齐：传播器从单位姿态起算、重力取 (0,0,-G)，因此
            // @c imuDisplacement 活在**首帧 IMU(body) 系**；而 @c poses 来自
            // 三角化与 PnP，活在**首帧相机系**。两者差一个外参旋转 R_bc。
            // 不转过来，线性系统会把 90° 的坐标系差当成重力偏差吸收掉，
            // 解出的尺度与姿态都会系统性偏掉（且不会报任何错）。
            const Eigen::Matrix3d cameraFromBody = bodyFromCamera_.toRotationMatrix().transpose();
            for (domain::Vec3 &displacement : imuDisplacement)
            {
                displacement = cameraFromBody * displacement;
            }
            const Eigen::Vector3d gravityReferenceCamera =
                cameraFromBody * Eigen::Vector3d(0.0, 0.0, -gravityMagnitude);
            const double maxSpeed = 20.0;

            const auto tangentBasis = [](const Eigen::Vector3d &normal, Eigen::Vector3d &t1, Eigen::Vector3d &t2)
            {
                const Eigen::Vector3d seed =
                    std::abs(normal.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                t1 = (seed - seed.dot(normal) * normal).normalized();
                t2 = normal.cross(t1);
            };

            Eigen::Vector3d gravityDirection = gravityReferenceCamera.normalized();
            Eigen::MatrixXd systemA;
            Eigen::VectorXd systemB;
            Eigen::VectorXd solution;
            const auto rows = static_cast<Eigen::Index>(3 * (frames.size() - 1));

            const auto solveLinear = [&]() -> bool
            {
                Eigen::Vector3d t1;
                Eigen::Vector3d t2;
                tangentBasis(gravityDirection, t1, t2);

                systemA = Eigen::MatrixXd::Zero(rows, 6);
                systemB = Eigen::VectorXd::Zero(rows);
                for (std::size_t frameIndex = 1; frameIndex < frames.size(); ++frameIndex)
                {
                    const double dt = stamps[frameIndex] - stamps[0];
                    if (!(dt > 1e-3))
                    {
                        result.detail = std::format("第 {} 帧与首帧时间间隔 {:.6f}s 太小", frameIndex, dt);
                        return false;
                    }
                    const auto row = static_cast<Eigen::Index>(3 * (frameIndex - 1));
                    systemA.block<3, 1>(row, 0) = poses[frameIndex].position;
                    systemA.block<3, 3>(row, 1) = -dt * Eigen::Matrix3d::Identity();
                    systemA.block<3, 1>(row, 4) = -0.5 * dt * dt * t1;
                    systemA.block<3, 1>(row, 5) = -0.5 * dt * dt * t2;
                    // Δp 是用配置重力积分出来的，已包含 ½dt²·g_ref；
                    // 这里只补「真实重力 − 配置重力」的差，否则重力会被
                    // 计两次（表现为尺度被放大数倍）。
                    systemB.segment<3>(row) =
                        imuDisplacement[frameIndex] +
                        0.5 * dt * dt * (gravityMagnitude * gravityDirection - gravityReferenceCamera);
                }

                const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(systemA);
                // 6 个未知量；秩不足说明激励不够，直接失败而不是给出勉强的解
                if (decomposition.rank() < 6)
                {
                    result.detail =
                        std::format("线性系统秩 {} / 6，观测不足以分离尺度、速度与重力", decomposition.rank());
                    return false;
                }
                solution = decomposition.solve(systemB);
                return solution.allFinite();
            };

            // 迭代更新重力方向，但只在残差真的下降时接受。
            //
            // 为什么需要这层保护：窗口短、旋转少时重力方向几乎不可观，
            // 切平面上的 w1/w2 会被噪声放大，更新后的 ĝ 可能跳到很远的
            // 方向并对下一次求解产生正反馈。工程上宁可停在第一步
            // （它等价于「重力固定为配置值」这个保守假设），也不能发散。
            bool solved = false;
            double bestResidual = std::numeric_limits<double>::infinity();
            Eigen::VectorXd bestSolution;
            Eigen::Vector3d bestGravityDirection = gravityDirection;

            for (int iteration = 0; iteration < 3; ++iteration)
            {
                if (!solveLinear())
                {
                    return result;
                }
                const double residualNorm = (systemA * solution - systemB).norm();
                if (residualNorm < bestResidual)
                {
                    bestResidual = residualNorm;
                    bestSolution = solution;
                    bestGravityDirection = gravityDirection;
                    solved = true;
                }
                else
                {
                    // 本次更新让拟合变差：回退到上一次的解并停止
                    break;
                }

                Eigen::Vector3d t1;
                Eigen::Vector3d t2;
                tangentBasis(gravityDirection, t1, t2);
                const Eigen::Vector3d gravity =
                    gravityMagnitude * gravityDirection + solution(4) * t1 + solution(5) * t2;
                if (!gravity.allFinite() || gravity.norm() < 1e-6)
                {
                    break;
                }
                const Eigen::Vector3d candidate = gravity.normalized();
                // 单步限制：一次更新不允许把重力方向转动超过 30°
                const double stepAngle = std::acos(std::clamp(candidate.dot(gravityDirection), -1.0, 1.0));
                if (stepAngle > 30.0 * std::numbers::pi / 180.0)
                {
                    break;
                }
                gravityDirection = candidate;
            }
            if (!solved)
            {
                return result;
            }
            solution = bestSolution;
            gravityDirection = bestGravityDirection;

            const double scale = solution(0);
            const domain::Vec3 velocityCamera = solution.segment<3>(1);

            if (!(scale > minScale) || !std::isfinite(scale) || !velocityCamera.allFinite())
            {
                result.detail = std::format("解出的尺度 {:.4e} 或速度非有限", scale);
                return result;
            }
            if (velocityCamera.norm() > maxSpeed)
            {
                result.detail = std::format("解出的初始速度 {:.2f} m/s 不合理", velocityCamera.norm());
                return result;
            }

            // ---------------------------------------------------------------
            // 8. 世界系对齐（重力模长已由约束精确保证，不再需要归一化修正）
            // ---------------------------------------------------------------
            const domain::Vec3 gravityCamera = domain::Vec3(gravityMagnitude * gravityDirection);

            // 世界系 z 轴对齐「重力反方向」，与静止初始化保持同一约定
            const domain::Vec3 upInCamera = -gravityCamera.normalized();
            const Eigen::Quaterniond worldFromCamera =
                Eigen::Quaterniond::FromTwoVectors(upInCamera, domain::Vec3(0.0, 0.0, 1.0));

            result.state.timestamp = stamps[0];
            result.state.pose = domain::Pose{worldFromCamera, domain::Vec3::Zero()};
            result.state.velocity = worldFromCamera * velocityCamera;
            result.state.bias = estimatedBias;

            // 初始状态协方差：由线性系统残差估噪声尺度，再取
            // @c σ²(AᵀA)⁻¹ 的重力块与速度块。
            //
            // 这是从数据里导出来的量，不是经验常数：残差大（拟合差）时
            // 初始状态就应当被下游更谨慎地对待。重力块的迹换算成倾角
            // 不确定度；航向仍然不可观，显式给大。位置是人为锚定的世界系
            // 原点，取一个紧值。
            {
                const Eigen::VectorXd residual = systemA * solution - systemB;
                // 6 个未知量（s、v0³、w1、w2）
                const double dof = std::max(1.0, static_cast<double>(rows) - 6.0);
                const double sigma2 = residual.squaredNorm() / dof;
                const Eigen::MatrixXd normal = systemA.transpose() * systemA;
                const Eigen::MatrixXd covariance = normal.completeOrthogonalDecomposition().pseudoInverse() * sigma2;
                if (covariance.allFinite())
                {
                    // 重力只剩切平面两个方向（模长被约束固定）
                    const double gravityVariance = covariance.block<2, 2>(4, 4).trace();
                    const double tiltSigma = std::clamp(
                        std::sqrt(std::max(0.0, gravityVariance)) / std::max(1e-6, imu_.gravityMagnitude), 1e-3, 0.2);
                    constexpr double kYawSigmaRad = 0.5;
                    constexpr double kOriginSigma = 1e-3;
                    domain::Mat6 poseCovariance = domain::Mat6::Zero();
                    poseCovariance(0, 0) = tiltSigma * tiltSigma;
                    poseCovariance(1, 1) = tiltSigma * tiltSigma;
                    poseCovariance(2, 2) = kYawSigmaRad * kYawSigmaRad;
                    poseCovariance(3, 3) = kOriginSigma * kOriginSigma;
                    poseCovariance(4, 4) = kOriginSigma * kOriginSigma;
                    poseCovariance(5, 5) = kOriginSigma * kOriginSigma;
                    result.poseCovariance = poseCovariance;
                    mylog::debug("[动态初始化] 初始协方差: 倾角 σ={:.4f} rad, 位置 σ={:.5f} m", tiltSigma,
                                 kOriginSigma);
                }
            }

            result.success = true;
            result.framesUsed = frames.size();
            result.inliers = static_cast<std::size_t>(inliers);
            result.detail = std::format("尺度 {:.3f}, 初始速度 {:.3f} m/s, 陀螺零偏 {:.4f} rad/s, 累计旋转 {:.1f}°",
                                        scale, result.state.velocity.norm(), estimatedBias.gyro.norm(), spannedDegrees);

            mylog::info("[动态初始化] 成功: {}", result.detail);
            return result;
        }

    private:
        domain::InitializationConfig initialization_;
        Eigen::Quaterniond bodyFromCamera_{Eigen::Quaterniond::Identity()};
        domain::ImuConfig imu_;
        cv::Mat K_;
        double focal_;
    };

} // namespace myslam::vio
