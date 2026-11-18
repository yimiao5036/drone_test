/**
 * @file hdmi_display_backend_selftest.cpp
 * @brief HdmiDisplayBackend 开发者自检单元测试（不依赖真实 DRM 设备）
 *
 * 目的：在**任意开发机**（含无 /dev/dri 的 WSL2）上验证 HDMI 直显后端对
 * VideoSender 的接口契约，覆盖：
 * - 注入工厂 HdmiMakeBackendFactory / 配置桥接 HdmiConfigFromEncoderConfig；
 * - Start/Stop 生命周期与幂等性、Open 失败时的降级；
 * - EncodeFrame 的帧校验（格式/尺寸/句柄）、stride 透传；
 * - kBusy 只丢帧不计错误、kFailed 计错误、成功累计；
 * - 统计量映射（SentFrameCount / ErrorCount / FramePrepareLatency /
 *   PacketWriteLatency / SkippedFrameCount / ActiveModeName）。
 *
 * 之所以能脱离硬件：libdrm/libgbm/EGL 只出现在 hdmi_drm_display.cpp 内，
 * 后端主体通过 IHdmiDisplay 抽象与设备解耦，测试注入 MockHdmiDisplay 即可。
 *
 * 构建与运行（在 WSL2 Ubuntu 中）：
 *   cmake -S videoPart/hdmi_part/src/hdmi -B build_hdmi -DDRONE_HAVE_HDMI_KMS=ON
 *   cmake --build build_hdmi -j$(nproc)
 *   ctest --test-dir build_hdmi --output-on-failure
 */
#include "hdmi/hdmi_display_backend.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>  // ::access，用于判断真实 DRM 设备是否存在

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

namespace drone::video_transmission {
namespace {

using common::LatencySummary;
using video::FrameBuffer;
using video::FrameHandle;
using video::PixelFormat;
using video::VideoFrameInfo;

/// 调用计数：由测试与 Mock 共享，使测试能在后端析构后仍读到计数。
struct CallCounters {
    int open = 0;
    int close = 0;
    int submit = 0;
};

/// 注入用 Mock 显示设备：不接触任何图形设备，行为可配置。
class MockHdmiDisplay final : public IHdmiDisplay {
public:
    explicit MockHdmiDisplay(int open_calls = 0)
        : counters_(std::make_shared<CallCounters>()) {
        counters_->open = open_calls;
    }

    /// 与测试共享的计数对象（后端析构后依然有效）。
    [[nodiscard]] std::shared_ptr<CallCounters> Counters() const { return counters_; }

    bool Open(const HdmiDisplayConfig& config) override {
        ++counters_->open;
        last_config = config;
        is_open_ = open_result;
        return open_result;
    }

    void Close() override {
        ++counters_->close;
        is_open_ = false;
    }

    [[nodiscard]] bool IsOpen() const override { return is_open_; }

    HdmiSubmitResult Submit(const std::uint8_t* data, std::uint32_t width,
                           std::uint32_t height, std::uint32_t hor_stride) override {
        ++counters_->submit;
        last_data = data;
        last_width = width;
        last_height = height;
        last_stride = hor_stride;
        return submit_result;
    }

    [[nodiscard]] LatencySummary FlipLatency() const override { return flip; }
    [[nodiscard]] LatencySummary PrepareLatency() const override { return prepare; }
    [[nodiscard]] std::uint64_t SkippedFrameCount() const override { return skipped; }
    [[nodiscard]] std::string ActiveModeName() const override { return mode_name; }

    // ---- 可配置行为 ----
    bool open_result = true;
    HdmiSubmitResult submit_result = HdmiSubmitResult::kSubmitted;
    LatencySummary flip{};
    LatencySummary prepare{};
    std::uint64_t skipped = 0;
    std::string mode_name = "1280x720@30(mock)";

    // ---- 调用记录 ----
    const std::uint8_t* last_data = nullptr;
    std::uint32_t last_width = 0;
    std::uint32_t last_height = 0;
    std::uint32_t last_stride = 0;
    HdmiDisplayConfig last_config{};

private:
    std::shared_ptr<CallCounters> counters_;
    bool is_open_ = false;
};

/// 构造一帧可提交的 NV12 句柄。
/// 像素缓冲由本对象持有（FrameBuffer 只借用其地址），故本对象需保活到提交结束。
class TestFrame {
public:
    explicit TestFrame(std::uint32_t width, std::uint32_t height,
                       std::uint32_t hor_stride = 0,
                       PixelFormat format = PixelFormat::kYuv420SpNv12)
        : hor_stride_(hor_stride == 0 ? width : hor_stride) {
        // NV12 占用 = stride × height × 1.5
        const std::size_t size =
            static_cast<std::size_t>(hor_stride_) * height * 3 / 2;
        storage_.assign(size, std::byte{0x40});

        VideoFrameInfo info;
        info.width = width;
        info.height = height;
        info.hor_stride = hor_stride_;
        info.ver_stride = height;
        info.buf_size = size;
        info.format = format;
        // recycler 传 nullptr：外部缓冲，析构时不归还内存池
        auto buffer = std::make_shared<FrameBuffer>(info, storage_.data(), size,
                                                   nullptr, 0);
        handle_ = FrameHandle(std::move(buffer));
    }

    [[nodiscard]] const FrameHandle& Handle() const { return handle_; }
    [[nodiscard]] const std::uint8_t* Data() const {
        return reinterpret_cast<const std::uint8_t*>(storage_.data());
    }
    [[nodiscard]] std::uint32_t HorStride() const { return hor_stride_; }

private:
    std::uint32_t hor_stride_;
    std::vector<std::byte> storage_;
    FrameHandle handle_;
};

/// 测试夹具：静音 spdlog，避免逐帧错误日志（节流后仍会有输出）干扰测试输出。
class HdmiBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        previous_level_ = spdlog::get_level();
        spdlog::set_level(spdlog::level::off);
    }
    void TearDown() override { spdlog::set_level(previous_level_); }

private:
    spdlog::level::level_enum previous_level_ = spdlog::level::info;
};

// ---------------------------------------------------------------------------
// 装配辅助：注入工厂与配置桥接
// ---------------------------------------------------------------------------

/// 工厂产出可用后端：返回非空、类型正确、未启动。
TEST_F(HdmiBackendTest, FactoryProducesUsableBackend) {
    HdmiDisplayConfig config;
    auto factory = HdmiMakeBackendFactory(config);
    ASSERT_TRUE(static_cast<bool>(factory));

    std::unique_ptr<IVideoEncoderBackend> backend = factory();
    ASSERT_NE(backend, nullptr);
    EXPECT_FALSE(backend->IsRunning());
    EXPECT_EQ(backend->SentFrameCount(), 0U);
    EXPECT_EQ(backend->ErrorCount(), 0U);
    backend->Stop();  // 未启动时停止必须安全
}

/// 工厂必须是 VideoSenderConfig::backend_factory 的同一类型，可直接赋值。
TEST_F(HdmiBackendTest, FactoryTypeMatchesVideoSenderInjectionPoint) {
    HdmiDisplayConfig config;
    std::function<std::unique_ptr<IVideoEncoderBackend>()> factory =
        HdmiMakeBackendFactory(config);
    ASSERT_TRUE(static_cast<bool>(factory));
    EXPECT_NE(factory(), nullptr);
}

/// 非法配置在工厂内被收敛为 nullptr，不向 VideoSender 抛异常。
TEST_F(HdmiBackendTest, FactoryReturnsNullOnInvalidConfig) {
    HdmiDisplayConfig bad;
    bad.width = 0;  // 分辨率 0 非法
    auto factory = HdmiMakeBackendFactory(bad);
    EXPECT_EQ(factory(), nullptr);
}

/// 配置桥接只取 width/height/fps，并把刷新率归到 30/60 档。
TEST_F(HdmiBackendTest, ConfigBridgeMapsSizeAndFps) {
    EncoderBackendConfig encode;
    encode.width = 1920;
    encode.height = 1080;
    encode.fps = 25;
    const HdmiDisplayConfig config = HdmiConfigFromEncoderConfig(encode);
    EXPECT_EQ(config.width, 1920U);
    EXPECT_EQ(config.height, 1080U);
    EXPECT_EQ(config.refresh_hz, 30);

    encode.fps = 60;
    EXPECT_EQ(HdmiConfigFromEncoderConfig(encode).refresh_hz, 60);

    // 源未给出尺寸时保持 HDMI 默认值，而不是被清零
    const EncoderBackendConfig empty;
    const HdmiDisplayConfig fallback = HdmiConfigFromEncoderConfig(empty);
    EXPECT_EQ(fallback.width, 1280U);
    EXPECT_EQ(fallback.height, 720U);
}

/// 静态配置非法时构造即抛异常，由装配层决定如何处理。
///
/// 用辅助函数而非直接写 EXPECT_THROW(HdmiDisplayBackend(cfg), ...)：
/// 后者会被解析成"声明了一个名为 cfg 的对象"（most vexing parse），
/// 实际去调用默认构造函数，既编不过也测不到目标行为。
TEST_F(HdmiBackendTest, InvalidConfigThrows) {
    const auto ctor_rejects = [](const HdmiDisplayConfig& config) -> bool {
        try {
            HdmiDisplayBackend backend(config);
            (void)backend;
            return false;  // 未抛异常：配置校验缺失
        } catch (const std::invalid_argument&) {
            return true;
        }
    };

    HdmiDisplayConfig zero_width;
    zero_width.width = 0;
    EXPECT_TRUE(ctor_rejects(zero_width));

    HdmiDisplayConfig zero_height;
    zero_height.height = 0;
    EXPECT_TRUE(ctor_rejects(zero_height));

    HdmiDisplayConfig zero_refresh;
    zero_refresh.refresh_hz = 0;
    EXPECT_TRUE(ctor_rejects(zero_refresh));

    HdmiDisplayConfig zero_timeout;
    zero_timeout.flip_timeout_ms = 0;
    EXPECT_TRUE(ctor_rejects(zero_timeout));

    // 走默认显示设备（display 为空）时，设备节点为空属非法
    HdmiDisplayConfig no_device;
    no_device.drm_device.clear();
    EXPECT_TRUE(ctor_rejects(no_device));

    // 反例：合法配置不得抛异常
    EXPECT_FALSE(ctor_rejects(HdmiDisplayConfig{}));
}

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

/// Start 把配置透传给显示设备，且重复调用是幂等的。
TEST_F(HdmiBackendTest, StartForwardsConfigAndIsIdempotent) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayConfig config;
    config.connector_name = "HDMI-A-1";
    config.color_space = HdmiColorSpace::kBt601;
    HdmiDisplayBackend backend(config, std::move(mock));

    ASSERT_TRUE(backend.Start());
    EXPECT_TRUE(backend.IsRunning());
    EXPECT_EQ(raw->Counters()->open, 1);
    EXPECT_EQ(raw->last_config.drm_device, config.drm_device);
    EXPECT_EQ(raw->last_config.width, 1280U);
    EXPECT_EQ(raw->last_config.height, 720U);
    EXPECT_EQ(raw->last_config.refresh_hz, 30);
    EXPECT_EQ(raw->last_config.connector_name, "HDMI-A-1");
    EXPECT_EQ(raw->last_config.color_space, HdmiColorSpace::kBt601);

    EXPECT_TRUE(backend.Start());  // 幂等：不再打开设备
    EXPECT_EQ(raw->Counters()->open, 1);
    EXPECT_EQ(backend.ErrorCount(), 0U);
}

/// 设备打开失败：Start 返回 false、不计为已运行、错误数 +1。
TEST_F(HdmiBackendTest, StartFailsWhenDeviceOpenFails) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    mock->open_result = false;
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    EXPECT_FALSE(backend.Start());
    EXPECT_FALSE(backend.IsRunning());
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(raw->Counters()->open, 1);
}

/// Stop 幂等，且会关闭已打开的设备。
TEST_F(HdmiBackendTest, StopIsIdempotent) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());
    backend.Stop();
    backend.Stop();
    EXPECT_FALSE(backend.IsRunning());
    EXPECT_EQ(raw->Counters()->close, 1);
}

/// 停止后再次 Start 可重新打开设备（对应 VideoSender 的启停流程）。
TEST_F(HdmiBackendTest, RestartAfterStopReopensDevice) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());
    backend.Stop();
    ASSERT_TRUE(backend.Start());
    EXPECT_EQ(raw->Counters()->open, 2);
    EXPECT_EQ(raw->Counters()->close, 1);
}

/// 析构时若仍在运行，必须自动关闭设备（不泄漏 DRM master / EGL 上下文）。
TEST_F(HdmiBackendTest, DestructorClosesRunningDevice) {
    std::shared_ptr<CallCounters> counters;
    {
        auto mock = std::make_unique<MockHdmiDisplay>();
        counters = mock->Counters();
        HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
        ASSERT_TRUE(backend.Start());
        EXPECT_EQ(counters->close, 0);
    }
    EXPECT_EQ(counters->close, 1);
}

// ---------------------------------------------------------------------------
// 帧校验与提交
// ---------------------------------------------------------------------------

/// 未启动就提交：拒绝并计错误，不触碰显示设备。
TEST_F(HdmiBackendTest, EncodeBeforeStartIsRejected) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    TestFrame frame(1280, 720);
    EXPECT_FALSE(backend.EncodeFrame(frame.Handle()));
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(backend.SentFrameCount(), 0U);
    EXPECT_EQ(raw->Counters()->submit, 0);
}

/// 空句柄：拒绝并计错误。
TEST_F(HdmiBackendTest, EncodeRejectsEmptyHandle) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    const FrameHandle empty;
    EXPECT_FALSE(backend.EncodeFrame(empty));
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(raw->Counters()->submit, 0);
}

/// 非 NV12 帧：拒绝并计错误（后端只接收 NV12，不做格式转换）。
TEST_F(HdmiBackendTest, EncodeRejectsNonNv12Format) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame rgb(1280, 720, 0, PixelFormat::kRgb888);
    EXPECT_FALSE(backend.EncodeFrame(rgb.Handle()));
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(raw->Counters()->submit, 0);
}

/// 尺寸与输出模式不一致：拒绝并计错误（宁可丢帧也不静默拉伸）。
TEST_F(HdmiBackendTest, EncodeRejectsSizeMismatch) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame smaller(640, 480);
    EXPECT_FALSE(backend.EncodeFrame(smaller.Handle()));
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(raw->Counters()->submit, 0);
}

/// 合法帧：透传尺寸与 hor_stride（stride 可大于 width，由显示实现按行处理）。
TEST_F(HdmiBackendTest, EncodeForwardsStrideToDisplay) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame frame(1280, 720, 1344);  // 行对齐填充：stride > width
    EXPECT_TRUE(backend.EncodeFrame(frame.Handle()));
    EXPECT_EQ(raw->Counters()->submit, 1);
    EXPECT_EQ(raw->last_width, 1280U);
    EXPECT_EQ(raw->last_height, 720U);
    EXPECT_EQ(raw->last_stride, 1344U);
    EXPECT_EQ(raw->last_data, frame.Data());
    EXPECT_EQ(backend.SentFrameCount(), 1U);
    EXPECT_EQ(backend.ErrorCount(), 0U);
}

/// kBusy（上次翻转未完成）：只丢帧，不计错误、不累计上屏。
TEST_F(HdmiBackendTest, BusyFrameIsDroppedWithoutError) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    mock->submit_result = HdmiSubmitResult::kBusy;
    mock->skipped = 5;
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame frame(1280, 720);
    EXPECT_FALSE(backend.EncodeFrame(frame.Handle()));
    EXPECT_EQ(backend.ErrorCount(), 0U);
    EXPECT_EQ(backend.SentFrameCount(), 0U);
    EXPECT_EQ(backend.SkippedFrameCount(), 5U);
    EXPECT_EQ(raw->Counters()->submit, 1);
}

/// kFailed（设备错误）：计错误，不计上屏。
TEST_F(HdmiBackendTest, FailedSubmitCountsError) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    mock->submit_result = HdmiSubmitResult::kFailed;
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame frame(1280, 720);
    EXPECT_FALSE(backend.EncodeFrame(frame.Handle()));
    EXPECT_EQ(backend.ErrorCount(), 1U);
    EXPECT_EQ(backend.SentFrameCount(), 0U);
    EXPECT_EQ(raw->Counters()->submit, 1);
}

/// 成功帧逐帧累计：连续提交与丢帧混合时计数正确。
TEST_F(HdmiBackendTest, SentCountAccumulatesAcrossMixedResults) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    TestFrame frame(1280, 720);
    int sent = 0;
    for (int i = 0; i < 10; ++i) {
        // 注意：mock 已被 move 进后端，此处必须用 raw 访问（用 mock 会空指针解引用）
        raw->submit_result = (i % 3 == 2) ? HdmiSubmitResult::kBusy
                                           : HdmiSubmitResult::kSubmitted;
        if (backend.EncodeFrame(frame.Handle())) {
            ++sent;
        }
    }
    EXPECT_EQ(raw->Counters()->submit, 10);
    EXPECT_EQ(backend.SentFrameCount(), static_cast<std::uint64_t>(sent));
    EXPECT_EQ(backend.ErrorCount(), 0U);  // kBusy 不算错误
}

/// 大量非法帧：错误计数不丢，且节流日志不会崩溃。
TEST_F(HdmiBackendTest, RepeatedInvalidFramesCountEveryError) {
    auto mock = std::make_unique<MockHdmiDisplay>();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    const FrameHandle empty;
    for (int i = 0; i < 250; ++i) {
        EXPECT_FALSE(backend.EncodeFrame(empty));
    }
    EXPECT_EQ(backend.ErrorCount(), 250U);
}

// ---------------------------------------------------------------------------
// 统计量映射：确认语义没有串位
// ---------------------------------------------------------------------------

/// FramePrepareLatency ↔ 显示设备准备耗时；PacketWriteLatency ↔ 翻转提交耗时。
/// 两者故意给不同数值，防止映射写反。
TEST_F(HdmiBackendTest, LatencySummariesAreNotSwapped) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    mock->prepare.p50_ms = 1.25;
    mock->prepare.p99_ms = 3.5;
    mock->flip.p50_ms = 12.5;
    mock->flip.p99_ms = 40.0;

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());

    const LatencySummary prepare = backend.FramePrepareLatency();
    const LatencySummary write = backend.PacketWriteLatency();
    EXPECT_DOUBLE_EQ(prepare.p50_ms, 1.25);
    EXPECT_DOUBLE_EQ(prepare.p99_ms, 3.5);
    EXPECT_DOUBLE_EQ(write.p50_ms, 12.5);
    EXPECT_DOUBLE_EQ(write.p99_ms, 40.0);
}

/// 未启动时查询统计量必须安全：不崩溃，且反映显示设备的当前状态。
/// 这里把 Mock 置为"未产生任何数据"的初始态，验证透传结果为 0/空。
TEST_F(HdmiBackendTest, LatencyQueriesAreSafeBeforeStart) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    mock->skipped = 0;
    mock->mode_name.clear();  // 尚未 modeset，无生效模式
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));

    EXPECT_FALSE(backend.IsRunning());
    EXPECT_EQ(raw->Counters()->open, 0);  // 构造不打开设备
    EXPECT_EQ(backend.SkippedFrameCount(), 0U);
    EXPECT_TRUE(backend.ActiveModeName().empty());
    EXPECT_EQ(backend.FramePrepareLatency().total_count, 0U);
    EXPECT_EQ(backend.PacketWriteLatency().total_count, 0U);
}

/// 模式名与丢帧计数直接取显示设备的实时值（供探针与日志使用）。
TEST_F(HdmiBackendTest, ModeNameAndSkippedCountAreForwarded) {
    auto mock = std::make_unique<MockHdmiDisplay>();
    MockHdmiDisplay* raw = mock.get();

    HdmiDisplayBackend backend(HdmiDisplayConfig{}, std::move(mock));
    ASSERT_TRUE(backend.Start());
    EXPECT_EQ(backend.ActiveModeName(), "1280x720@30(mock)");

    raw->skipped = 42;
    raw->mode_name = "1280x720@60";
    EXPECT_EQ(backend.SkippedFrameCount(), 42U);
    EXPECT_EQ(backend.ActiveModeName(), "1280x720@60");
}

// ---------------------------------------------------------------------------
// 真实设备冒烟：仅在存在 DRM 设备时运行（香橙派），开发机自动跳过
// ---------------------------------------------------------------------------

/// 真实 DRM 设备生命周期冒烟。开发机（无 /dev/dri）自动跳过。
TEST_F(HdmiBackendTest, RealDrmDeviceLifecycleSmoke) {
    HdmiDisplayConfig config;
    if (::access(config.drm_device.c_str(), R_OK) != 0) {
        GTEST_SKIP() << "无 DRM 设备节点 " << config.drm_device
                     << "（开发机正常现象），本用例需在香橙派上执行";
    }

    HdmiDisplayBackend backend(config);
    ASSERT_TRUE(backend.Start()) << "有设备节点时 Start 必须成功";
    EXPECT_TRUE(backend.IsRunning());
    EXPECT_FALSE(backend.ActiveModeName().empty());

    TestFrame frame(config.width, config.height);
    // 连续提交：允许 kBusy 丢帧（源比输出快时属预期），但不允许失败
    for (int i = 0; i < 10; ++i) {
        backend.EncodeFrame(frame.Handle());
    }
    EXPECT_EQ(backend.ErrorCount(), 0U);

    backend.Stop();
    EXPECT_FALSE(backend.IsRunning());
}

}  // namespace
}  // namespace drone::video_transmission
