#pragma once

/**
 * @file mei_undistort.hpp
 * @brief 把 MEI 鱼眼原图重映射成虚拟针孔画幅，供可视化对照。
 *
 * @c MeiFeatureTracker 输出的特征点已经是「去畸变后的虚拟针孔像素」，
 * 直接叠加在鱼眼原图上必然错位。本文件按与跟踪器完全相同的模型链路
 * 反向生成整幅图像：
 *
 *   虚拟针孔像素 (u, v)
 *     -> 归一化光线方向 (nx, ny)                  // 与跟踪器的 mx/mz 同源
 *     -> 除以 (1 + xi*‖n‖) 得到去畸变归一化坐标   // MEI 投影的逆
 *     -> radtan 畸变 + 鱼眼内参 -> 原始鱼眼像素
 *
 * 注意不能用 @c (u-cx)/fx 直接当 MEI 反投影的输入：跟踪器的针孔坐标是
 * @c f*mx/mz+c，而 Mei 反投影的输入是去畸变归一化坐标 @c mx/(mz+xi*‖m‖)，
 * 两者差一个 @c 1/(1+xi*‖n‖) 的缩放。混用会让整幅图被压到中心。
 * 映射表只在构造时计算一次（640x480 约 0.3 M 个采样点），之后每帧只做
 * 一次 @c cv::remap，开销可以忽略。落在原图外的采样点标记为无效，
 * remap 时按边框常量填黑。
 */

#include "dataset/configHolder.hpp"
#include <cmath>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace myslam::dataset
{

    /**
     * @brief MEI 鱼眼 -> 虚拟针孔图像的去畸变器
     */
    class MeiUndistorter
    {
    public:
        explicit MeiUndistorter(const MeiIntrinsics &intr)
            : intr_(intr),
              mapX_(intr.imageHeight, intr.imageWidth, CV_32FC1),
              mapY_(intr.imageHeight, intr.imageWidth, CV_32FC1)
        {
            buildMaps();
        }

        /**
         * @brief 把一帧鱼眼原图重映射为虚拟针孔图像
         *
         * @param raw 原始灰度（或 BGR）图，尺寸必须与内参一致
         * @return 同尺寸的去畸变图；输入为空或尺寸不符时返回空 Mat
         */
        cv::Mat apply(const cv::Mat &raw) const
        {
            if (raw.empty() || raw.cols != intr_.imageWidth || raw.rows != intr_.imageHeight)
            {
                return {};
            }
            cv::Mat out;
            // 无效采样点的映射值停在 -1，remap 越界后按 borderValue 填黑。
            cv::remap(raw, out, mapX_, mapY_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
            return out;
        }

        /// 映射表落在原图内的采样点比例（用于日志与自检）
        double validRatio() const { return validRatio_; }

    private:
        void buildMaps()
        {
            std::size_t valid = 0;
            for (int v = 0; v < mapY_.rows; ++v)
            {
                float *rowX = mapX_.ptr<float>(v);
                float *rowY = mapY_.ptr<float>(v);
                for (int u = 0; u < mapX_.cols; ++u)
                {
                    double rawX = -1.0;
                    double rawY = -1.0;
                    if (projectToRaw(static_cast<double>(u), static_cast<double>(v), rawX, rawY))
                    {
                        ++valid;
                    }
                    rowX[u] = static_cast<float>(rawX);
                    rowY[u] = static_cast<float>(rawY);
                }
            }
            const auto total = static_cast<double>(mapX_.total());
            validRatio_ = total > 0.0 ? static_cast<double>(valid) / total : 0.0;
        }

        /**
         * @brief 虚拟针孔像素 -> 原始鱼眼像素
         *
         * @return false 表示该像素没有对应的鱼眼采样点（视线在相机后方或越界）
         */
        bool projectToRaw(double u, double v, double &outX, double &outY) const
        {
            // 虚拟针孔像素 -> 归一化光线方向（跟踪器输出的 uvNorm 就是 (mx/mz, my/mz)）
            const double nx = (u - intr_.pinholeCx()) / intr_.pinholeFx();
            const double ny = (v - intr_.pinholeCy()) / intr_.pinholeFy();

            // MEI 投影的逆：π(m) = m_xy / (mz + xi*‖m‖)，对方向向量取 m = (nx, ny, 1)
            // 后化为 (nx, ny) / (1 + xi*‖n‖)。这一步的除以 (1 + xi*‖n‖) 不能省——
            // 漏掉它会让映射整体收缩，矫正图只剩中心一块。
            const double directionNorm = std::sqrt(1.0 + nx * nx + ny * ny);
            const double w = 1.0 + intr_.xi * directionNorm;
            if (!(w > 1e-6))
            {
                return false; // 视场边界之外
            }
            const double xp = nx / w;
            const double yp = ny / w;
            const double rp2 = xp * xp + yp * yp;

            // radtan（与跟踪器的 D = [k1, k2, p1, p2, 0] 一致）
            const double radial = 1.0 + intr_.k1 * rp2 + intr_.k2 * rp2 * rp2;
            const double xpp = xp * radial + 2.0 * intr_.p1 * xp * yp + intr_.p2 * (rp2 + 2.0 * xp * xp);
            const double ypp = yp * radial + intr_.p1 * (rp2 + 2.0 * yp * yp) + 2.0 * intr_.p2 * xp * yp;
            if (!std::isfinite(xpp) || !std::isfinite(ypp))
            {
                return false;
            }

            outX = intr_.fx * xpp + intr_.cx;
            outY = intr_.fy * ypp + intr_.cy;
            if (!std::isfinite(outX) || !std::isfinite(outY))
            {
                return false;
            }
            return outX >= 0.0 && outX < static_cast<double>(intr_.imageWidth) && outY >= 0.0 &&
                   outY < static_cast<double>(intr_.imageHeight);
        }

        MeiIntrinsics intr_;
        cv::Mat mapX_;
        cv::Mat mapY_;
        double validRatio_{0.0};
    };

} // namespace myslam::dataset
