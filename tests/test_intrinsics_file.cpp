/**
 * @file test_intrinsics_file.cpp
 * @brief 内参估计文件解析的单元测试
 *
 * 覆盖 @c myslam::dataset::loadIntrinsicsFile 的正常路径与拒绝路径：格式正确
 * 时把针孔估计映射成可直接使用的 MeiIntrinsics（xi 与畸变置零、缩放置一），
 * 任何能导致“用错内参跑完整条序列”的输入都必须被拒绝。断言计数风格与
 * mylog tests 一致。
 */

#include "dataset/intrinsics_file.hpp"
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

    /// 写入临时文件并返回路径
    std::filesystem::path writeTemp(const std::string &name, const std::string &content)
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
        std::ofstream out(path);
        out << content;
        return path;
    }

    void testValidPinhole()
    {
        const auto path = writeTemp("myslam_intr_valid.txt", "# comment line\n"
                                                             "\n"
                                                             "model = pinhole\n"
                                                             "source = mast3r\n"
                                                             "fx = 461.5\n"
                                                             "fy = 460.0\n"
                                                             "cx = 320\n"
                                                             "cy = 240\n"
                                                             "width = 640\n"
                                                             "height = 480\n"
                                                             "pairs = 6\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(result.has_value(), "格式正确的估计文件应解析成功");
        if (result)
        {
            const auto &in = result->intrinsics;
            expectTrue(in.fx == 461.5, "fx 应被读入");
            expectTrue(in.fy == 460.0, "fy 应被读入");
            expectTrue(in.cx == 320.0 && in.cy == 240.0, "主点应被读入");
            expectTrue(in.imageWidth == 640 && in.imageHeight == 480, "画幅应被读入");
            expectTrue(in.xi == 0.0, "针孔估计必须把 xi 置零");
            expectTrue(in.k1 == 0.0 && in.k2 == 0.0 && in.p1 == 0.0 && in.p2 == 0.0, "针孔估计必须清除畸变");
            expectTrue(in.pinholeScale == 1.0, "针孔估计的缩放应为 1");
            expectTrue(in.isValid(), "解析结果应通过 isValid");
            expectTrue(result->source == "mast3r", "source 应被保留");
            expectTrue(result->model == "pinhole", "model 应被保留");
            // pairs 是估计工具追加的日志字段，应被当作未知键忽略而不是失败
            expectTrue(result->warnings.size() == 1, "未知键应只产生一条告警");
        }
        std::filesystem::remove(path);
    }

    void testMissingPrincipalPoint()
    {
        const auto path = writeTemp("myslam_intr_nop.png.txt", "fx = 400\n"
                                                               "fy = 400\n"
                                                               "width = 800\n"
                                                               "height = 600\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(result.has_value(), "缺少 cx/cy 时应回退到图像中心而不是失败");
        if (result)
        {
            expectTrue(result->intrinsics.cx == 400.0, "cx 应回退到 width/2");
            expectTrue(result->intrinsics.cy == 300.0, "cy 应回退到 height/2");
            expectTrue(result->warnings.size() == 2, "缺 cx/cy 应各有一条告警");
            expectTrue(result->source == "file", "未给 source 时应有默认标记");
        }
        std::filesystem::remove(path);
    }

    void testMissingFile()
    {
        const auto result = myslam::dataset::loadIntrinsicsFile("/nonexistent/myslam_intr.txt");
        expectTrue(!result.has_value(), "不存在的文件应失败");
    }

    void testMissingFocal()
    {
        const auto path = writeTemp("myslam_intr_nofx.txt", "fy = 400\n"
                                                            "cx = 320\n"
                                                            "cy = 240\n"
                                                            "width = 640\n"
                                                            "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "缺少 fx 应失败");
        std::filesystem::remove(path);
    }

    void testNonPositiveFocal()
    {
        const auto path = writeTemp("myslam_intr_negfx.txt", "fx = -400\n"
                                                             "fy = 400\n"
                                                             "width = 640\n"
                                                             "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "负焦距应失败");
        std::filesystem::remove(path);
    }

    void testMalformedNumber()
    {
        const auto path = writeTemp("myslam_intr_badnum.txt", "fx = 4o0\n"
                                                              "fy = 400\n"
                                                              "width = 640\n"
                                                              "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "无法解析的数字应失败");
        std::filesystem::remove(path);
    }

    void testUnsupportedModel()
    {
        const auto path = writeTemp("myslam_intr_mei.txt", "model = mei\n"
                                                           "fx = 758\n"
                                                           "fy = 676\n"
                                                           "width = 640\n"
                                                           "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "非 pinhole 模型应被拒绝");
        std::filesystem::remove(path);
    }

    void testPrincipalPointOutside()
    {
        const auto path = writeTemp("myslam_intr_pp.txt", "fx = 400\n"
                                                          "fy = 400\n"
                                                          "cx = 900\n"
                                                          "cy = 240\n"
                                                          "width = 640\n"
                                                          "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "画幅外的主点应被拒绝");
        std::filesystem::remove(path);
    }

    void testMissingLineSeparator()
    {
        const auto path = writeTemp("myslam_intr_nosep.txt", "fx 400\n"
                                                             "fy = 400\n"
                                                             "width = 640\n"
                                                             "height = 480\n");
        const auto result = myslam::dataset::loadIntrinsicsFile(path);
        expectTrue(!result.has_value(), "缺少 '=' 的行应导致缺少 fx 从而失败");
        std::filesystem::remove(path);
    }
} // namespace

int main()
{
    testValidPinhole();
    testMissingPrincipalPoint();
    testMissingFile();
    testMissingFocal();
    testNonPositiveFocal();
    testMalformedNumber();
    testUnsupportedModel();
    testPrincipalPointOutside();
    testMissingLineSeparator();

    if (g_failures == 0)
    {
        std::printf("OK: %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d/%d checks\n", g_failures, g_checks);
    return 1;
}
