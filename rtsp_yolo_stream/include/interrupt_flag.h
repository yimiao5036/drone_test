#ifndef RTSP_YOLO_STREAM_INTERRUPT_FLAG_H
#define RTSP_YOLO_STREAM_INTERRUPT_FLAG_H

#include <atomic>
#include <cstdint>

// 跨模块共享的网络中断 / 看门狗状态。
// 全局唯一实例定义在 main.cpp 中，rtsp_decoder / rtsp_encoder 通过本头文件访问。
namespace rtsp_stream {

// 置 true 后，拉流/推流的阻塞网络 I/O 会尽快返回（用于超时中断与 Ctrl+C 唤醒）。
// 每次重连成功后由主循环清除。
extern std::atomic<bool> g_net_interrupt;

// 距上次成功读到帧的时间点（steady_clock 毫秒），-1 表示尚未成功读到任何帧。
// 由看门狗线程读取，主循环在每帧成功后更新。
extern std::atomic<int64_t> g_last_frame_ms;

// FFmpeg AVIOInterruptCB 回调：返回非 0 表示中断阻塞中的网络 I/O
inline int NetInterruptCallback(void*) {
    return g_net_interrupt.load() ? 1 : 0;
}

} // namespace rtsp_stream

#endif // RTSP_YOLO_STREAM_INTERRUPT_FLAG_H
