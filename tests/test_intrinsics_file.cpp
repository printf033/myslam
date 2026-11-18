/**
 * @file test_intrinsics_file.cpp
 * @brief 内参估计文件解析的单元测试
 *
 * 覆盖 @c myslam::dataset::loadIntrinsicsFile 的正常路径与拒绝路径。核心语义：
 * 文件提供的是**虚拟针孔焦距** @c focal，畸变参数（xi / k1 / k2 / p1 / p2）
 * 从基线标定继承，从而保留鱼眼几何。任何能导致“用错内参跑完整条序列”的
 * 输入都必须被拒绝。断言计数风格与 mylog tests 一致。
 */

#include "sensor/intrinsics_file.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
#include "config/lua_config.hpp"
#endif

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

    /// 在仓库根 / 构建目录两种当前目录下定位随仓库分发的配置
    std::filesystem::path findShippedConfig(const char *relative)
    {
        for (const char *prefix : {"", "../", "../../"})
        {
            const std::filesystem::path candidate = std::filesystem::path(prefix) / relative;
            if (std::filesystem::is_regular_file(candidate))
            {
                return candidate;
            }
        }
        return {};
    }

    /// 测试基线：MEI 标定从随仓库分发的 lua 配置读取，源码里不内联任何数据源数值。
    ///
    /// 选 SubT 配置是因为它是真实的 MEI 鱼眼标定（xi 与畸变均非零），
    /// 这才能验证「文件缺省字段继承基线」在鱼眼几何下真的生效。
    /// 无 Lua 运行时（MYSLAM_ENABLE_LUA=OFF）时退回代码默认值，并在 stderr 说明：
    /// 此时 xi 继承变为退化（0 == 0），鱼眼几何下不具判别力。
    /// 实际加载基线标定（只在首次调用时执行一次，断言也只计一次）
    myslam::dataset::MeiIntrinsics loadCalibratedBaseline()
    {
#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
        const auto path = findShippedConfig("config/subt_handheld1.lua");
        // 有 Lua 时不允许静默退化：找不到配置说明工作目录不对，直接报失败，
        // 否则鱼眼继承检查会在默认值下变成无判别力的空壳。
        expectTrue(!path.empty(), "找到 config/subt_handheld1.lua（需从仓库根或构建目录运行）");
        if (!path.empty())
        {
            myslam::config::ProjectConfig project;
            const auto loaded = myslam::config::load(path, project, /*applyLogging=*/false);
            expectTrue(loaded.has_value(), "从 lua 配置读取基线内参");
            if (loaded)
            {
                expectTrue(project.intrinsics.isMei(), "基线配置为 MEI 模型");
                expectTrue(project.intrinsics.xi > 0.0, "基线配置的 xi 非零（鱼眼继承才具判别力）");
                return project.intrinsics;
            }
        }
#else
        std::fprintf(stderr, "[warning] 无 Lua 运行时，基线内参退回代码默认值（鱼眼继承检查退化）\n");
#endif
        myslam::dataset::MeiIntrinsics intrinsics{};
        intrinsics.model = myslam::camera::CameraModel::MEI;
        return intrinsics;
    }

    /// 各用例共用同一份基线（避免重复读配置与重复计数断言）
    const myslam::dataset::MeiIntrinsics &calibratedBaseline()
    {
        static const myslam::dataset::MeiIntrinsics baseline = loadCalibratedBaseline();
        return baseline;
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
        const auto baseline = calibratedBaseline();
        const auto result = myslam::dataset::loadIntrinsicsFile(path, baseline);
        expectTrue(result.has_value(), "focal 格式的文件应解析成功");
        if (result)
        {
            const auto &in = result->intrinsics;
            // 关键断言：虚拟针孔焦距等于文件给出的 focal
            expectTrue(near(in.pinholeFx(), 408.0, 1e-9), "pinholeFx() 应等于 focal");
            expectTrue(near(in.fx, baseline.fx, 1e-6), "MEI 投影参数 fx 应继承标定值");
            expectTrue(near(in.xi, baseline.xi, 1e-9), "xi 应继承标定值，不被清零");
            expectTrue(near(in.k1, baseline.k1, 1e-12), "k1 应继承标定值");
            expectTrue(near(in.k2, baseline.k2, 1e-12), "k2 应继承标定值");
            expectTrue(near(in.pinholeScale, 408.0 / baseline.fx, 1e-9), "pinholeScale 应为 focal/fx");
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
        const auto baseline = calibratedBaseline();
        const auto result = myslam::dataset::loadIntrinsicsFile(path, baseline);
        expectTrue(result.has_value(), "显式给出畸变参数时应解析成功");
        if (result)
        {
            expectTrue(near(result->intrinsics.xi, 1.0, 1e-12), "xi 应被文件覆盖");
            expectTrue(near(result->intrinsics.k1, -0.1, 1e-12), "k1 应被文件覆盖");
            // 未给出的 k2 仍继承
            expectTrue(near(result->intrinsics.k2, baseline.k2, 1e-12), "k2 未给出时应继承");
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
            // focal 优先：scale 被重新算成 focal / 基线 fx，而不是 0.9
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

    /**
     * @brief 针孔基线：focal 直接就是 fx/fy，且 MEI 的球面参数必须被归一化掉
     *
     * 基线故意构造成「模型为针孔、却携带 MEI 参数」——这正是从 SubT 标定
     * 切到针孔序列时的真实状态。若不清零 xi，前端会把针孔图当作鱼眼反解。
     * 用带非零 xi、非单位缩放、非方形像素的基线，上述每一条都有区分度。
     */
    void testPinholeBaselineNormalizesMeiParameters()
    {
        myslam::dataset::MeiIntrinsics baseline = calibratedBaseline();
        baseline.model = myslam::camera::CameraModel::PINHOLE;
        baseline.pinholeScale = 0.5;
        const double aspect = baseline.fy / baseline.fx;

        const auto path = writeTemp("myslam_intr_pinhole.txt", "focal = 400\n"
                                                               "width = 800\n"
                                                               "height = 600\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path, baseline);
        expectTrue(result.has_value(), "针孔基线内参文件应解析成功");
        if (result)
        {
            const auto &in = result->intrinsics;
            expectTrue(!in.isMei(), "模型保持针孔");
            expectTrue(near(in.xi, 0.0, 1e-12), "针孔下继承的 xi 被归一化为 0");
            expectTrue(near(in.pinholeScale, 1.0, 1e-12), "针孔下 pinholeScale 归一化为 1");
            expectTrue(near(in.fx, 400.0, 1e-9), "针孔下 focal 直接作为 fx");
            expectTrue(near(in.pinholeFx(), 400.0, 1e-9), "针孔下虚拟针孔焦距等于 fx");
            expectTrue(near(in.fy, 400.0 * aspect, 1e-9), "针孔下 fy 保留基线的长宽比");
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
    testPinholeBaselineNormalizesMeiParameters();

    if (g_failures == 0)
    {
        std::printf("OK: %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d/%d checks\n", g_failures, g_checks);
    return 1;
}
