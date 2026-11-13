#pragma once

/**
 * @file rosbag2_source.hpp
 * @brief rosbag2（.db3 / SQLite）数据源：topic 盘点、CDR 解码、IMU 与参考轨迹。
 *
 * rosbag2 的 .db3 本质是一个 SQLite 数据库：
 * @code
 * topics(id, name, type, serialization_format)
 * messages(id, topic_id, timestamp, data)   -- data 是 CDR 序列化后的 BLOB
 * @endcode
 *
 * 本文件只负责把 DB 行变成领域数据，不做任何图像解码（sensor 层不依赖
 * OpenCV）：压缩流原样返回 JPEG 字节，未压缩图返回原始像素平面与行跨度。
 * 上层（回放/诊断/标定）自行决定是否解码。
 *
 * @par CDR 对齐
 *
 * ROS2 的 CDR 序列化以 4 字节封装头（`00 01 00 00`）开始，之后所有字段的
 * 对齐都相对 **载荷起点**（即封装头之后）。字符串编码为「u32 长度（含结尾
 * NUL）+ 字节 + 补齐到 4」。本项目实测的 bag 里，raw `sensor_msgs/Image`
 * 在 encoding 之后 **没有** `is_bigendian` 字段，而标准定义有；解码器对两种
 * 布局按长度自洽性自适应。
 *
 * @par IMU 来源
 *
 * 本项目数据集的 IMU 不是 `sensor_msgs/Imu`，而是 mfosn 组合导航输出的
 * `std_msgs/String` JSON，字段 `angular_rate_dps`（deg/s）与
 * `acceleration_mps2`（机体系，含重力）。解析时换算为 rad/s 与 m/s^2。
 */

#include "domain/types.hpp"
#include "logger.hpp"
#include "sensor/subt_dataset.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <limits>
#include <nlohmann/json.hpp>
#include <numbers>
#include <optional>
#include <span>
#include <sqlite3.h>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 一个 topic 的盘点信息
     */
    struct TopicInfo
    {
        int id{0};
        std::string name;
        std::string type;
        std::uint64_t count{0};
    };

    /**
     * @brief 一条参考位姿（来自 PoseStamped）
     */
    struct PoseRecord
    {
        double timestamp{0.0}; ///< 相对秒
        domain::Vec3 position = domain::Vec3::Zero();
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    };

    /**
     * @brief 一帧图像（未解码的原始字节）
     */
    struct ImageFrame
    {
        double timestamp{0.0}; ///< 相对秒
        std::string encoding;  ///< 压缩流保留原始 format 串；否则为像素编码（如 "bgr8"）
        int width{0};
        int height{0};
        int step{0}; ///< 原始图每行字节数；压缩流为 0
        bool compressed{false};
        std::vector<std::uint8_t> data;
    };

    /**
     * @brief 相机内参（来自 sensor_msgs/CameraInfo）
     */
    struct CameraIntrinsics
    {
        int width{0};
        int height{0};
        std::string distortionModel;
        std::vector<double> distortion;
        std::array<double, 9> k{};
        std::array<double, 12> p{};
    };

    namespace detail
    {

        /**
         * @brief 小端 CDR 读取器：偏移相对载荷起点（4 字节封装头之后）
         */
        class CdrReader
        {
        public:
            explicit CdrReader(std::span<const std::uint8_t> payload)
                : data_(payload)
            {
            }

            [[nodiscard]] bool ok() const { return ok_; }
            [[nodiscard]] std::size_t offset() const { return offset_; }
            [[nodiscard]] std::size_t remaining() const { return ok_ ? data_.size() - offset_ : 0; }

            void seek(std::size_t offset)
            {
                offset_ = offset;
                ok_ = offset_ <= data_.size();
            }

            void align(std::size_t n)
            {
                const std::size_t mask = n - 1;
                offset_ = (offset_ + mask) & ~mask;
                if (offset_ > data_.size())
                {
                    ok_ = false;
                }
            }

            std::uint8_t u8()
            {
                std::uint8_t v = 0;
                take(&v, sizeof(v));
                return v;
            }

            std::uint32_t u32()
            {
                align(4);
                std::uint32_t v = 0;
                take(&v, sizeof(v));
                return v;
            }

            std::int32_t i32()
            {
                align(4);
                std::int32_t v = 0;
                take(&v, sizeof(v));
                return v;
            }

            double f64()
            {
                align(8);
                double v = 0.0;
                take(&v, sizeof(v));
                return v;
            }

            std::string str()
            {
                const std::uint32_t len = u32();
                if (!ok_ || len == 0)
                {
                    return {};
                }
                if (offset_ + len > data_.size())
                {
                    ok_ = false;
                    return {};
                }
                std::string out(reinterpret_cast<const char *>(data_.data() + offset_), len - 1);
                offset_ += len;
                align(4);
                return out;
            }

            std::span<const std::uint8_t> bytes(std::size_t n)
            {
                if (offset_ + n > data_.size())
                {
                    ok_ = false;
                    return {};
                }
                const auto out = data_.subspan(offset_, n);
                offset_ += n;
                return out;
            }

        private:
            void take(void *dst, std::size_t n)
            {
                if (!ok_ || offset_ + n > data_.size())
                {
                    ok_ = false;
                    return;
                }
                std::memcpy(dst, data_.data() + offset_, n);
                offset_ += n;
            }

            std::span<const std::uint8_t> data_;
            std::size_t offset_{0};
            bool ok_{true};
        };

        /// 去掉 4 字节封装头，返回 CDR 载荷（对齐基准）
        inline std::span<const std::uint8_t> cdrPayload(const std::uint8_t *data, std::size_t size)
        {
            if (data == nullptr || size < 4)
            {
                return {};
            }
            return {data + 4, size - 4};
        }

        struct Header
        {
            std::int32_t sec{0};
            std::uint32_t nsec{0};
            std::string frameId;
        };

        inline Header decodeHeader(CdrReader &r)
        {
            Header h;
            h.sec = r.i32();
            h.nsec = r.u32();
            h.frameId = r.str();
            return h;
        }

        inline std::size_t bytesPerPixel(std::string_view encoding)
        {
            if (encoding == "bgr8" || encoding == "rgb8")
            {
                return 3;
            }
            if (encoding == "bgra8" || encoding == "rgba8")
            {
                return 4;
            }
            if (encoding == "mono8" || encoding == "8UC1")
            {
                return 1;
            }
            if (encoding == "mono16" || encoding == "16UC1")
            {
                return 2;
            }
            if (encoding == "32FC1")
            {
                return 4;
            }
            return 0;
        }

        /// PoseStamped: Header → align(8) → 7 doubles
        inline std::optional<PoseRecord> decodePoseStamped(std::span<const std::uint8_t> payload, double timestamp)
        {
            if (payload.empty())
            {
                return std::nullopt;
            }
            CdrReader r(payload);
            (void)decodeHeader(r);
            r.align(8);
            PoseRecord rec;
            rec.timestamp = timestamp;
            // 必须逐个读取：在同一表达式里多次调用有副作用的 r.f64()，
            // 实参求值顺序未指定（GCC 从右到左），会把 x/z 读反。
            const double px = r.f64();
            const double py = r.f64();
            const double pz = r.f64();
            rec.position = domain::Vec3(px, py, pz);
            const double qx = r.f64();
            const double qy = r.f64();
            const double qz = r.f64();
            const double qw = r.f64();
            if (!r.ok())
            {
                return std::nullopt;
            }
            rec.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
            return rec;
        }

        /// CompressedImage: Header → string format → u32 data_size → bytes
        inline std::optional<ImageFrame> decodeCompressedImage(std::span<const std::uint8_t> payload, double timestamp)
        {
            if (payload.empty())
            {
                return std::nullopt;
            }
            CdrReader r(payload);
            (void)decodeHeader(r);
            const std::string format = r.str();
            const std::uint32_t dataSize = r.u32();
            const auto pixels = r.bytes(dataSize);
            if (!r.ok() || r.remaining() != 0)
            {
                return std::nullopt;
            }
            ImageFrame frame;
            frame.timestamp = timestamp;
            frame.encoding = format;
            frame.compressed = true;
            frame.data.assign(pixels.begin(), pixels.end());
            return frame;
        }

        /**
         * @brief raw Image: Header → u32 h → u32 w → string encoding → [is_bigendian?] → u32 step → u32 size → data
         *
         * 本项目 bag 省略了 is_bigendian；标准 sensor_msgs/Image 有 1 字节。
         * 先按「省略」读 step/size，再用 size == remaining、step == width*bpp
         * 校验；不成立则回退到「有 is_bigendian」布局。
         */
        inline std::optional<ImageFrame> decodeRawImage(std::span<const std::uint8_t> payload, double timestamp)
        {
            if (payload.empty())
            {
                return std::nullopt;
            }
            CdrReader r(payload);
            (void)decodeHeader(r);
            const std::uint32_t height = r.u32();
            const std::uint32_t width = r.u32();
            const std::string encoding = r.str();
            if (!r.ok() || height == 0 || width == 0)
            {
                return std::nullopt;
            }

            const std::size_t bpp = bytesPerPixel(encoding);
            const std::size_t mark = r.offset();
            std::uint32_t step = r.u32();
            std::uint32_t dataSize = r.u32();
            bool compact = r.ok() && dataSize == r.remaining();
            if (compact && bpp != 0 && step != 0 && step != static_cast<std::uint32_t>(width * bpp))
            {
                compact = false;
            }
            if (!compact)
            {
                r.seek(mark);
                (void)r.u8(); // is_bigendian
                r.align(4);
                step = r.u32();
                dataSize = r.u32();
                if (!r.ok() || dataSize != r.remaining())
                {
                    return std::nullopt;
                }
            }
            const auto pixels = r.bytes(dataSize);
            if (!r.ok())
            {
                return std::nullopt;
            }

            ImageFrame frame;
            frame.timestamp = timestamp;
            frame.encoding = encoding;
            frame.compressed = false;
            frame.width = static_cast<int>(width);
            frame.height = static_cast<int>(height);
            frame.step = static_cast<int>(step);
            frame.data.assign(pixels.begin(), pixels.end());
            return frame;
        }

        /// CameraInfo: Header → u32 h → u32 w → string model → align8 → u32 dcount → align8 → D[] → K[9] → R[9] → P[12]
        inline std::optional<CameraIntrinsics> decodeCameraInfo(std::span<const std::uint8_t> payload)
        {
            if (payload.empty())
            {
                return std::nullopt;
            }
            CdrReader r(payload);
            (void)decodeHeader(r);
            const std::uint32_t height = r.u32();
            const std::uint32_t width = r.u32();
            const std::string model = r.str();
            r.align(8);
            const std::uint32_t dCount = r.u32();
            r.align(8);
            if (!r.ok() || dCount > 64)
            {
                return std::nullopt;
            }
            CameraIntrinsics info;
            info.width = static_cast<int>(width);
            info.height = static_cast<int>(height);
            info.distortionModel = model;
            info.distortion.resize(dCount);
            for (double &d : info.distortion)
            {
                d = r.f64();
            }
            for (double &k : info.k)
            {
                k = r.f64();
            }
            for (int i = 0; i < 9; ++i)
            {
                (void)r.f64(); // R 跳过
            }
            for (double &p : info.p)
            {
                p = r.f64();
            }
            if (!r.ok())
            {
                return std::nullopt;
            }
            return info;
        }

    } // namespace detail

    /**
     * @brief rosbag2 数据源：topic 盘点、IMU、参考轨迹与按需取图
     *
     * 用法：
     * @code
     * RosBagSource src("/path/to/xxx.db3");
     * if (auto rc = src.load(); !rc) { ... }
     * for (const auto &t : src.topics()) { ... }
     * const auto &imu = src.imu();
     * @endcode
     *
     * 图像不预载：`load()` 只记录每个图像 topic 的 rowid 与时间戳，
     * `imageAt()` 再按 rowid 取 BLOB，内存占用与图像数量无关。
     */
    class RosBagSource
    {
    public:
        explicit RosBagSource(std::filesystem::path db3Path)
            : db3Path_(std::move(db3Path))
        {
        }

        ~RosBagSource() { close(); }

        RosBagSource(const RosBagSource &) = delete;
        RosBagSource &operator=(const RosBagSource &) = delete;

        /**
         * @brief 打开数据库、盘点 topic、解析 IMU 与参考轨迹
         * @return 成功时无值；文件不存在或 SQLite 错误时返回具体原因
         */
        std::expected<void, DatasetError> load()
        {
            if (const auto rc = openDatabase(); !rc)
            {
                return rc;
            }
            if (const auto rc = readTopics(); !rc)
            {
                return rc;
            }
            if (const auto rc = scanMessages(); !rc)
            {
                return rc;
            }
            if (const auto rc = loadImu(); !rc)
            {
                return rc;
            }
            if (const auto rc = loadPoses(); !rc)
            {
                return rc;
            }
            return {};
        }

        /// 全部 topic（含消息计数）
        [[nodiscard]] const std::vector<TopicInfo> &topics() const { return topics_; }

        /// 消息总行数
        [[nodiscard]] std::uint64_t messageCount() const { return messageCount_; }

        /// IMU 记录（时间相对化，角速度已换算为 rad/s）
        [[nodiscard]] const std::vector<ImuRecord> &imu() const { return imu_; }

        /// 参考位姿（ins_pose 优先，否则 dvl/pose）；无则为空
        [[nodiscard]] const std::vector<PoseRecord> &referencePoses() const { return poses_; }

        /// 被选中的 IMU topic 名；无则空
        [[nodiscard]] const std::string &imuTopicName() const { return imuTopic_; }

        /// 被选中的参考位姿 topic 名；无则空
        [[nodiscard]] const std::string &poseTopicName() const { return poseTopic_; }

        /// 解析 IMU 时跳过的坏消息数（JSON 缺字段或解码失败）
        [[nodiscard]] std::uint64_t skippedImu() const { return skippedImu_; }

        /// 解析参考位姿时跳过/失败的消息数
        [[nodiscard]] std::uint64_t skippedPose() const { return skippedPose_; }

        /// 所有图像 topic 名
        [[nodiscard]] std::vector<std::string> imageTopics() const
        {
            std::vector<std::string> out;
            for (const auto &t : topics_)
            {
                if (isImageType(t.type))
                {
                    out.push_back(t.name);
                }
            }
            return out;
        }

        /// 所有 CameraInfo topic 名
        [[nodiscard]] std::vector<std::string> cameraInfoTopics() const
        {
            std::vector<std::string> out;
            for (const auto &t : topics_)
            {
                if (t.type == "sensor_msgs/msg/CameraInfo")
                {
                    out.push_back(t.name);
                }
            }
            return out;
        }

        /// 指定图像 topic 的帧数；topic 不存在返回 0
        [[nodiscard]] std::size_t imageCount(std::string_view topic) const
        {
            const TopicRows *rows = findImageRows(topic);
            return rows != nullptr ? rows->rowIds.size() : 0;
        }

        /// 指定图像 topic 的各帧相对时间戳（秒）
        [[nodiscard]] std::vector<double> imageStamps(std::string_view topic) const
        {
            std::vector<double> out;
            const TopicRows *rows = findImageRows(topic);
            if (rows == nullptr)
            {
                return out;
            }
            out.reserve(rows->timestampsNs.size());
            for (const std::int64_t ns : rows->timestampsNs)
            {
                out.push_back(relativeSeconds(ns));
            }
            return out;
        }

        /**
         * @brief 按序号读取一帧图像（解码后的元数据 + 原始字节）
         * @return 越界、SQLite 错误或 CDR 解码失败时返回具体原因
         */
        [[nodiscard]] std::expected<ImageFrame, DatasetError> imageAt(std::string_view topic, std::size_t index) const
        {
            const TopicRows *rows = findImageRows(topic);
            if (rows == nullptr)
            {
                return std::unexpected(DatasetError{std::format("图像 topic 不存在: {}", topic)});
            }
            if (index >= rows->rowIds.size())
            {
                return std::unexpected(DatasetError{std::format("图像下标越界: {} >= {}", index, rows->rowIds.size())});
            }
            const std::int64_t rowId = rows->rowIds[index];
            const double timestamp = relativeSeconds(rows->timestampsNs[index]);

            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT data FROM messages WHERE rowid = ?", -1, &stmt, nullptr) != SQLITE_OK)
            {
                return std::unexpected(DatasetError{std::format("查询图像失败: {}", sqlite3_errmsg(db))});
            }
            std::expected<ImageFrame, DatasetError> result{std::unexpected(DatasetError{"未读取到图像 BLOB"})};
            sqlite3_bind_int64(stmt, 1, rowId);
            if (sqlite3_step(stmt) == SQLITE_ROW)
            {
                const auto *blob = static_cast<const std::uint8_t *>(sqlite3_column_blob(stmt, 0));
                const int size = sqlite3_column_bytes(stmt, 0);
                const auto payload = detail::cdrPayload(blob, static_cast<std::size_t>(size));
                std::optional<ImageFrame> frame;
                if (rows->type == "sensor_msgs/msg/CompressedImage")
                {
                    frame = detail::decodeCompressedImage(payload, timestamp);
                }
                else
                {
                    frame = detail::decodeRawImage(payload, timestamp);
                }
                if (frame)
                {
                    result = std::move(*frame);
                }
                else
                {
                    result = std::unexpected(DatasetError{"图像 CDR 解码失败"});
                }
            }
            sqlite3_finalize(stmt);
            return result;
        }

        /// 读取指定 CameraInfo topic 的第一条内参
        [[nodiscard]] std::optional<CameraIntrinsics> cameraInfo(std::string_view topic) const
        {
            const int topicId = findTopicId(topic);
            if (topicId < 0)
            {
                return std::nullopt;
            }
            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT data FROM messages WHERE topic_id = ? LIMIT 1", -1, &stmt, nullptr) !=
                SQLITE_OK)
            {
                return std::nullopt;
            }
            sqlite3_bind_int(stmt, 1, topicId);
            std::optional<CameraIntrinsics> out;
            if (sqlite3_step(stmt) == SQLITE_ROW)
            {
                const auto *blob = static_cast<const std::uint8_t *>(sqlite3_column_blob(stmt, 0));
                const int size = sqlite3_column_bytes(stmt, 0);
                out = detail::decodeCameraInfo(detail::cdrPayload(blob, static_cast<std::size_t>(size)));
            }
            sqlite3_finalize(stmt);
            return out;
        }

        /// 由首尾时间戳估计的 IMU 平均频率 (Hz)；样本不足 2 条返回 0
        [[nodiscard]] double estimateImuRate() const
        {
            if (imu_.size() < 2)
            {
                return 0.0;
            }
            const double span = imu_.back().timestamp - imu_.front().timestamp;
            return span > 0.0 ? static_cast<double>(imu_.size() - 1) / span : 0.0;
        }

    private:
        struct TopicRows
        {
            int topicId{-1};
            std::string type;
            std::vector<std::int64_t> rowIds;
            std::vector<std::int64_t> timestampsNs;
        };

        static bool isImageType(std::string_view type)
        {
            return type == "sensor_msgs/msg/Image" || type == "sensor_msgs/msg/CompressedImage";
        }

        static std::expected<void, DatasetError> fail(std::string message)
        {
            mylog::error("{}", message);
            return std::unexpected(DatasetError{std::move(message)});
        }

        static std::string columnText(sqlite3_stmt *stmt, int column)
        {
            const auto *text = sqlite3_column_text(stmt, column);
            return text != nullptr ? std::string(reinterpret_cast<const char *>(text)) : std::string{};
        }

        [[nodiscard]] double relativeSeconds(std::int64_t ns) const
        {
            return static_cast<double>(ns - originNs_) * 1e-9;
        }

        [[nodiscard]] int findTopicId(std::string_view name) const
        {
            for (const auto &t : topics_)
            {
                if (t.name == name)
                {
                    return t.id;
                }
            }
            return -1;
        }

        [[nodiscard]] const TopicRows *findImageRows(std::string_view topic) const
        {
            const int topicId = findTopicId(topic);
            if (topicId < 0)
            {
                return nullptr;
            }
            const auto it = imageRows_.find(topicId);
            return it != imageRows_.end() ? &it->second : nullptr;
        }

        void close()
        {
            if (db_ != nullptr)
            {
                sqlite3_close(db_);
                db_ = nullptr;
            }
        }

        std::expected<void, DatasetError> openDatabase()
        {
            std::filesystem::path path = db3Path_;
            if (std::filesystem::is_directory(path))
            {
                // 传入目录时自动挑第一个 .db3
                std::error_code ec;
                for (const auto &entry : std::filesystem::directory_iterator(path, ec))
                {
                    if (entry.is_regular_file() && entry.path().extension() == ".db3")
                    {
                        path = entry.path();
                        break;
                    }
                }
            }
            if (!std::filesystem::is_regular_file(path))
            {
                return fail(std::format("rosbag2 数据库不存在: {}", path.string()));
            }
            const std::string native = path.string();
            if (sqlite3_open_v2(native.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
            {
                const std::string message =
                    std::format("无法打开 rosbag2 数据库: {} ({})", native, sqlite3_errmsg(db_));
                close();
                return fail(message);
            }
            return {};
        }

        std::expected<void, DatasetError> readTopics()
        {
            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT id, name, type FROM topics", -1, &stmt, nullptr) != SQLITE_OK)
            {
                return fail(std::format("读取 topics 表失败: {}", sqlite3_errmsg(db)));
            }
            while (sqlite3_step(stmt) == SQLITE_ROW)
            {
                TopicInfo info;
                info.id = sqlite3_column_int(stmt, 0);
                info.name = columnText(stmt, 1);
                info.type = columnText(stmt, 2);
                topics_.push_back(std::move(info));
            }
            sqlite3_finalize(stmt);
            if (topics_.empty())
            {
                return fail("rosbag2 数据库里没有任何 topic");
            }
            return {};
        }

        std::expected<void, DatasetError> scanMessages()
        {
            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT topic_id, rowid, timestamp FROM messages", -1, &stmt, nullptr) !=
                SQLITE_OK)
            {
                return fail(std::format("扫描 messages 表失败: {}", sqlite3_errmsg(db)));
            }
            std::unordered_map<int, std::size_t> idToIndex;
            for (std::size_t i = 0; i < topics_.size(); ++i)
            {
                idToIndex[topics_[i].id] = i;
            }
            originNs_ = std::numeric_limits<std::int64_t>::max();
            while (sqlite3_step(stmt) == SQLITE_ROW)
            {
                const int topicId = sqlite3_column_int(stmt, 0);
                const std::int64_t rowId = sqlite3_column_int64(stmt, 1);
                const std::int64_t timestamp = sqlite3_column_int64(stmt, 2);
                originNs_ = std::min(originNs_, timestamp);
                ++messageCount_;

                const auto it = idToIndex.find(topicId);
                if (it == idToIndex.end())
                {
                    continue;
                }
                TopicInfo &info = topics_[it->second];
                ++info.count;
                if (isImageType(info.type))
                {
                    TopicRows &rows = imageRows_[topicId];
                    rows.topicId = topicId;
                    rows.type = info.type;
                    rows.rowIds.push_back(rowId);
                    rows.timestampsNs.push_back(timestamp);
                }
            }
            sqlite3_finalize(stmt);
            if (originNs_ == std::numeric_limits<std::int64_t>::max())
            {
                return fail("messages 表为空");
            }
            return {};
        }

        std::expected<void, DatasetError> loadImu()
        {
            // 选 ins_data（std_msgs/String JSON）；没有就跳过
            for (const auto &t : topics_)
            {
                if (t.type == "std_msgs/msg/String" && t.name.find("ins_data") != std::string::npos)
                {
                    imuTopic_ = t.name;
                    imuTopicId_ = t.id;
                    break;
                }
            }
            if (imuTopicId_ < 0)
            {
                mylog::warn("[rosbag] 未找到 ins_data（IMU）topic");
                return {};
            }

            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", -1,
                                   &stmt, nullptr) != SQLITE_OK)
            {
                return fail(std::format("查询 IMU 失败: {}", sqlite3_errmsg(db)));
            }
            sqlite3_bind_int(stmt, 1, imuTopicId_);
            while (sqlite3_step(stmt) == SQLITE_ROW)
            {
                const std::int64_t timestamp = sqlite3_column_int64(stmt, 0);
                const auto *blob = static_cast<const std::uint8_t *>(sqlite3_column_blob(stmt, 1));
                const int size = sqlite3_column_bytes(stmt, 1);
                const auto payload = detail::cdrPayload(blob, static_cast<std::size_t>(size));
                detail::CdrReader r(payload);
                const std::string json = r.str();
                if (!r.ok() || json.empty())
                {
                    ++skippedImu_;
                    continue;
                }
                const auto doc = nlohmann::json::parse(json, nullptr, false);
                if (doc.is_discarded() || !doc.contains("angular_rate_dps") || !doc.contains("acceleration_mps2"))
                {
                    ++skippedImu_;
                    continue;
                }
                try
                {
                    const auto &g = doc.at("angular_rate_dps");
                    const auto &a = doc.at("acceleration_mps2");
                    ImuRecord rec;
                    rec.timestamp = relativeSeconds(timestamp);
                    constexpr double kDegToRad = std::numbers::pi / 180.0;
                    rec.gyro = domain::Vec3(g.at("x").get<double>(), g.at("y").get<double>(), g.at("z").get<double>()) *
                               kDegToRad;
                    rec.accel = domain::Vec3(a.at("x").get<double>(), a.at("y").get<double>(), a.at("z").get<double>());
                    imu_.push_back(rec);
                }
                catch (const nlohmann::json::exception &)
                {
                    ++skippedImu_;
                }
            }
            sqlite3_finalize(stmt);
            return {};
        }

        std::expected<void, DatasetError> loadPoses()
        {
            // 优先 ins_pose，其次任意 PoseStamped
            for (const auto &t : topics_)
            {
                if (t.type == "geometry_msgs/msg/PoseStamped" && t.name.find("ins_pose") != std::string::npos)
                {
                    poseTopic_ = t.name;
                    poseTopicId_ = t.id;
                    break;
                }
            }
            if (poseTopicId_ < 0)
            {
                for (const auto &t : topics_)
                {
                    if (t.type == "geometry_msgs/msg/PoseStamped" && t.name.find("pose") != std::string::npos)
                    {
                        poseTopic_ = t.name;
                        poseTopicId_ = t.id;
                        break;
                    }
                }
            }
            if (poseTopicId_ < 0)
            {
                mylog::warn("[rosbag] 未找到 PoseStamped 参考轨迹 topic");
                return {};
            }

            sqlite3 *db = db_;
            sqlite3_stmt *stmt = nullptr;
            if (sqlite3_prepare_v2(db, "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", -1,
                                   &stmt, nullptr) != SQLITE_OK)
            {
                return fail(std::format("查询参考轨迹失败: {}", sqlite3_errmsg(db)));
            }
            sqlite3_bind_int(stmt, 1, poseTopicId_);
            while (sqlite3_step(stmt) == SQLITE_ROW)
            {
                const std::int64_t timestamp = sqlite3_column_int64(stmt, 0);
                const auto *blob = static_cast<const std::uint8_t *>(sqlite3_column_blob(stmt, 1));
                const int size = sqlite3_column_bytes(stmt, 1);
                const auto payload = detail::cdrPayload(blob, static_cast<std::size_t>(size));
                const auto rec = detail::decodePoseStamped(payload, relativeSeconds(timestamp));
                if (!rec)
                {
                    ++skippedPose_;
                    continue;
                }
                poses_.push_back(*rec);
            }
            sqlite3_finalize(stmt);
            return {};
        }

        std::filesystem::path db3Path_;
        sqlite3 *db_{nullptr};
        std::vector<TopicInfo> topics_;
        std::unordered_map<int, TopicRows> imageRows_;
        std::vector<ImuRecord> imu_;
        std::vector<PoseRecord> poses_;
        std::string imuTopic_;
        std::string poseTopic_;
        int imuTopicId_{-1};
        int poseTopicId_{-1};
        std::int64_t originNs_{0};
        std::uint64_t messageCount_{0};
        std::uint64_t skippedImu_{0};
        std::uint64_t skippedPose_{0};
    };

} // namespace myslam::dataset
