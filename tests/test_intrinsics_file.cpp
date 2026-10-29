/**
 * @file test_intrinsics_file.cpp
 * @brief 内参估计文件解析的单元测试
 *
 * 覆盖 @c myslam::dataset::loadIntrinsicsFile 的正常路径与拒绝路径。核心语义：
 * 文件提供的是**虚拟针孔焦距** @c focal，畸变参数（xi / k1 / k2 / p1 / p2）
 * 从基线标定继承，从而保留鱼眼几何。任何能导致“用错内参跑完整条序列”的
 * 输入都必须被拒绝。断言计数风格与 mylog tests 一致。
 */

#include "dataset/intrinsics_file.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
    int g_checks = 0;
    int g_failures = 0;

    void expectTrue(bool condition, const char *what)
    {
        ++g_checks;
        if (!condition)
        {
            ++g_failures;
            std::fprintf(stderr, "FAIL: %s\n", what);
        }
    }

    /// 检查两个 double 在容差内相等
    bool near(double a, double b, double tolerance = 1e-9)
    {
        return std::abs(a - b) <= tolerance;
    }

    /// 写入临时文件并返回路径
    std::filesystem::path writeTemp(const std::string &name, const std::string &content)
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
        std::ofstream out(path);
        out << content;
        return path;
    }

    /// Handheld1 的标定内参，作为基线
    myslam::dataset::MeiIntrinsics calibratedBaseline()
    {
        return myslam::dataset::MeiIntrinsics{};
    }

    void testFocalWithInheritedDistortion()
    {
        const auto path = writeTemp("myslam_intr_focal.txt", "# estimated\n"
                                                             "model = pinhole\n"
                                                             "source = mast3r\n"
                                                             "focal = 408.0\n"
                                                             "cx = 320\n"
                                                             "cy = 240\n"
                                                             "width = 640\n"
                                                             "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(result.has_value(), "focal 格式的文件应解析成功");
        if (result)
        {
            const auto &in = result->intrinsics;
            // 关键断言：虚拟针孔焦距等于文件给出的 focal
            expectTrue(near(in.pinholeFx(), 408.0, 1e-9), "pinholeFx() 应等于 focal");
            expectTrue(near(in.fx, 758.3153257832925, 1e-6), "MEI 投影参数 fx 应继承标定值");
            expectTrue(near(in.xi, 1.583106303248484, 1e-9), "xi 应继承标定值，不被清零");
            expectTrue(near(in.k1, -0.059098218173967695, 1e-12), "k1 应继承标定值");
            expectTrue(near(in.k2, 0.1793477408661115, 1e-12), "k2 应继承标定值");
            expectTrue(near(in.pinholeScale, 408.0 / 758.3153257832925, 1e-9), "pinholeScale 应为 focal/fx");
            expectTrue(result->inheritedDistortion, "应标记畸变参数为继承");
            expectTrue(near(result->focalPx, 408.0, 1e-9), "focalPx 应回传");
            expectTrue(result->source == "mast3r", "source 应被保留");
        }
        std::filesystem::remove(path);
    }

    void testExplicitDistortionOverridesBaseline()
    {
        const auto path = writeTemp("myslam_intr_xi.txt", "focal = 500.0\n"
                                                          "xi = 1.0\n"
                                                          "k1 = -0.1\n"
                                                          "width = 640\n"
                                                          "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(result.has_value(), "显式给出畸变参数时应解析成功");
        if (result)
        {
            expectTrue(near(result->intrinsics.xi, 1.0, 1e-12), "xi 应被文件覆盖");
            expectTrue(near(result->intrinsics.k1, -0.1, 1e-12), "k1 应被文件覆盖");
            // 未给出的 k2 仍继承
            expectTrue(near(result->intrinsics.k2, 0.1793477408661115, 1e-12), "k2 未给出时应继承");
            expectTrue(!result->inheritedDistortion, "部分覆盖时不应标记为完全继承");
        }
        std::filesystem::remove(path);
    }

    void testMissingPrincipalPointFallsBackToCentre()
    {
        const auto path = writeTemp("myslam_intr_noc.txt", "focal = 400\nwidth = 800\nheight = 600\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(result.has_value(), "缺少 cx/cy 时应回退到图像中心");
        if (result)
        {
            expectTrue(near(result->intrinsics.cx, 400.0, 1e-9), "cx 应为 width/2");
            expectTrue(near(result->intrinsics.cy, 300.0, 1e-9), "cy 应为 height/2");
            expectTrue(result->warnings.size() == 2, "缺 cx/cy 应各有一条告警");
            expectTrue(result->source == "file", "未给 source 时应有默认标记");
        }
        std::filesystem::remove(path);
    }

    void testRejectsLegacyFxField()
    {
        // 早期实现把 MASt3R 的针孔焦距写成 fx，会把鱼眼当针孔处理
        const auto path = writeTemp("myslam_intr_legacy.txt", "model = pinhole\n"
                                                              "source = mast3r\n"
                                                              "fx = 408.0\n"
                                                              "fy = 408.0\n"
                                                              "width = 640\n"
                                                              "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "旧格式的 fx 字段应被拒绝而不是静默误用");
        std::filesystem::remove(path);
    }

    void testMissingFocal()
    {
        const auto path = writeTemp("myslam_intr_nofocal.txt", "width = 640\nheight = 480\nxi = 0\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "缺少 focal 应失败");
        std::filesystem::remove(path);
    }

    void testNonPositiveFocal()
    {
        const auto path = writeTemp("myslam_intr_negfocal.txt", "focal = -400\nwidth = 640\nheight = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "负焦距应失败");
        std::filesystem::remove(path);
    }

    void testMalformedNumber()
    {
        const auto path = writeTemp("myslam_intr_badnum.txt", "focal = 4o0\nwidth = 640\nheight = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "无法解析的数字应失败");
        std::filesystem::remove(path);
    }

    void testUnsupportedModel()
    {
        const auto path = writeTemp("myslam_intr_model.txt", "model = fisheye\n"
                                                             "focal = 400\n"
                                                             "width = 640\n"
                                                             "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "未知相机模型应被拒绝");
        std::filesystem::remove(path);
    }

    void testPrincipalPointOutside()
    {
        const auto path = writeTemp("myslam_intr_pp.txt", "focal = 400\n"
                                                          "cx = 900\n"
                                                          "cy = 240\n"
                                                          "width = 640\n"
                                                          "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(!result.has_value(), "画幅外的主点应被拒绝");
        std::filesystem::remove(path);
    }

    void testMissingFile()
    {
        const auto result = myslam::dataset::loadIntrinsicsFile("/nonexistent/myslam_intr.txt", calibratedBaseline());
        expectTrue(!result.has_value(), "不存在的文件应失败");
    }

    void testScaleConflictWarns()
    {
        const auto path = writeTemp("myslam_intr_scale.txt", "focal = 408\n"
                                                             "pinholeScale = 0.9\n"
                                                             "width = 640\n"
                                                             "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, calibratedBaseline());
        expectTrue(result.has_value(), "同时给出 scale 与 focal 时应成功");
        if (result)
        {
            // focal 优先：scale 被重新算成 408/758.3，而不是 0.9
            expectTrue(near(result->intrinsics.pinholeFx(), 408.0, 1e-9), "focal 应优先于 pinholeScale");
            bool warned = false;
            for (const auto &w : result->warnings)
            {
                warned = warned || w.find("pinholeScale") != std::string::npos;
            }
            expectTrue(warned, "冲突时应给出一条告警");
        }
        std::filesystem::remove(path);
    }
} // namespace

int main()
{
    testFocalWithInheritedDistortion();
    testExplicitDistortionOverridesBaseline();
    testMissingPrincipalPointFallsBackToCentre();
    testRejectsLegacyFxField();
    testMissingFocal();
    testNonPositiveFocal();
    testMalformedNumber();
    testUnsupportedModel();
    testPrincipalPointOutside();
    testMissingFile();
    testScaleConflictWarns();

    if (g_failures == 0)
    {
        std::printf("OK: %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d/%d checks\n", g_failures, g_checks);
    return 1;
}
