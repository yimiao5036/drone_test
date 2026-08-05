/**
 * @file topic.h
 * @brief 类型安全的发布订阅主题 Topic<Message>
 *
 * 属于 drone/common 模块，是各模块线程间解耦通信的基础设施。
 * 设计参考 PX4/ROS2 的发布订阅模型，纯标准库实现，不引入第三方依赖。
 * 详细使用说明见同目录下的 topic.md。
 */
#ifndef DRONE_COMMON_TOPIC_H_
#define DRONE_COMMON_TOPIC_H_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone::common {

/// 类型安全的发布订阅主题。
///
/// 设计要点：
/// - 消息类型在编译期确定，禁止 void* 和无类型字节指针。
/// - 每个订阅者拥有独立的有界队列，慢订阅者不会阻塞发布者或其他订阅者。
/// - 消息通过 std::shared_ptr<const Message> 分发，发布到多个订阅者时只复制
///   智能指针，不复制消息对象。视频帧可将内存池返回的 RAII 帧句柄直接作为
///   消息发布，底层缓冲在最后一个消费者释放句柄时自动归还内存池。
/// - 订阅句柄 Subscription 使用 RAII 管理生命周期，析构自动退订并清空队列。
/// - 主题 Close 后已入队消息仍可被取完，等待中的订阅者会被唤醒，支持确定性停机。
///
/// 线程安全约定：
/// - Publish / Subscribe / Close / SubscriberCount 可在任意线程并发调用。
/// - 每个 Subscription 是单消费者句柄（不可复制），仅允许一个消费者线程使用，
///   其上的 Take 系列操作无需再加锁。
///
/// 使用示例见 include/common/topic.md。
template <typename Message>
class Topic final {
    static_assert(!std::is_reference_v<Message>, "Topic message type cannot be a reference");

public:
    /// 消息句柄：不可变消息的共享所有权指针。多订阅者共享同一份消息对象。
    using MessagePtr = std::shared_ptr<const Message>;

    /// 订阅队列溢出策略。
    enum class OverflowPolicy {
        kDropOldest,  ///< 队列满时丢弃最旧消息，适合视频帧和最新无人机状态
        kRejectNewest ///< 队列满时拒绝最新消息，用于不能覆盖旧消息的场景
    };

    /// 一次 Publish 的统计结果，便于上层监控丢帧和订阅规模。
    struct PublishResult {
        bool accepted = false;              ///< 消息是否被主题接收并分发
        std::size_t subscriber_count = 0;   ///< 当前有效的订阅者数量
        std::size_t delivered_count = 0;    ///< 实际投递成功的订阅者数量
        std::size_t dropped_count = 0;      ///< 因队列满而被丢弃的消息数量
    };

private:
    /// 单个订阅者的内部状态：独立的互斥锁、条件变量和有界消息队列。
    struct SubscriberState {
        SubscriberState(std::size_t queue_capacity, OverflowPolicy overflow_policy)
            : capacity(queue_capacity), policy(overflow_policy) {}

        mutable std::mutex mutex;
        std::condition_variable condition;
        std::deque<MessagePtr> messages;
        const std::size_t capacity;
        const OverflowPolicy policy;
        bool closed = false;
        std::size_t dropped_count = 0;
    };

    /// 主题核心状态：订阅者注册表。通过 shared_ptr 持有，订阅句柄持有 weak_ptr，
    /// 保证主题析构时订阅句柄仍能安全读取（此时队列为空且已关闭）。
    struct Core {
        mutable std::mutex mutex;
        std::vector<std::weak_ptr<SubscriberState>> subscribers;
        bool closed = false;
    };

public:
    /// 单个订阅者的 RAII 句柄。
    ///
    /// Subscription 不可复制，避免多个消费者意外竞争同一个订阅队列；可以移动给
    /// 实际消费线程。句柄析构或 Reset 后自动关闭并清空自己的队列，主题会移除
    /// 该订阅者，后续消息不再投递。
    class Subscription final {
    public:
        Subscription() = default;
        ~Subscription() { Reset(); }

        Subscription(const Subscription&) = delete;
        Subscription& operator=(const Subscription&) = delete;

        Subscription(Subscription&& other) noexcept
            : core_(std::move(other.core_)), state_(std::move(other.state_)) {}

        Subscription& operator=(Subscription&& other) noexcept {
            if (this != &other) {
                Reset();
                core_ = std::move(other.core_);
                state_ = std::move(other.state_);
            }
            return *this;
        }

        /// 非阻塞获取一条消息；队列为空时返回 std::nullopt。
        /// 适合在轮询循环中使用，例如控制线程的固定周期快照。
        [[nodiscard]] std::optional<MessagePtr> TryTake() {
            const auto state = state_;
            if (!state) {
                return std::nullopt;
            }

            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->messages.empty()) {
                return std::nullopt;
            }

            auto message = std::move(state->messages.front());
            state->messages.pop_front();
            return message;
        }

        /// 阻塞等待消息，直到队列非空或主题被关闭。
        /// 主题关闭后会先取完已经入队的消息，再返回 std::nullopt。
        /// 适合独占消费线程（如 PX4 串口线程读取命令队列）。
        [[nodiscard]] std::optional<MessagePtr> WaitTake() {
            const auto state = state_;
            if (!state) {
                return std::nullopt;
            }

            std::unique_lock<std::mutex> lock(state->mutex);
            state->condition.wait(lock, [&state] {
                return state->closed || !state->messages.empty();
            });
            if (state->messages.empty()) {
                return std::nullopt;
            }

            auto message = std::move(state->messages.front());
            state->messages.pop_front();
            return message;
        }

        /// 在指定时间内等待消息；超时或主题已关闭且队列为空时返回 std::nullopt。
        /// 适合带超时的消费循环，例如看门狗周期检查。
        template <typename Rep, typename Period>
        [[nodiscard]] std::optional<MessagePtr> WaitTakeFor(
            const std::chrono::duration<Rep, Period>& timeout) {
            const auto state = state_;
            if (!state) {
                return std::nullopt;
            }

            std::unique_lock<std::mutex> lock(state->mutex);
            const bool ready = state->condition.wait_for(lock, timeout, [&state] {
                return state->closed || !state->messages.empty();
            });
            if (!ready || state->messages.empty()) {
                return std::nullopt;
            }

            auto message = std::move(state->messages.front());
            state->messages.pop_front();
            return message;
        }

        /// 订阅是否仍然有效（未关闭）。默认构造的句柄返回 false。
        [[nodiscard]] bool IsOpen() const {
            const auto state = state_;
            if (!state) {
                return false;
            }

            std::lock_guard<std::mutex> lock(state->mutex);
            return !state->closed;
        }

        /// 当前队列中待消费的消息数量。
        [[nodiscard]] std::size_t PendingCount() const {
            const auto state = state_;
            if (!state) {
                return 0;
            }

            std::lock_guard<std::mutex> lock(state->mutex);
            return state->messages.size();
        }

        /// 本订阅者因队列满累计丢弃的消息数量，用于监控慢消费者。
        [[nodiscard]] std::size_t DroppedCount() const {
            const auto state = state_;
            if (!state) {
                return 0;
            }

            std::lock_guard<std::mutex> lock(state->mutex);
            return state->dropped_count;
        }

        /// 主动取消订阅；可以重复调用，幂等。清空队列并通知等待中的消费者。
        void Reset() noexcept {
            auto state = std::move(state_);
            core_.reset();
            if (!state) {
                return;
            }

            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->closed = true;
                state->messages.clear();
            }
            state->condition.notify_all();
        }

    private:
        friend class Topic<Message>;

        Subscription(std::weak_ptr<Core> core, std::shared_ptr<SubscriberState> state)
            : core_(std::move(core)), state_(std::move(state)) {}

        std::weak_ptr<Core> core_;
        std::shared_ptr<SubscriberState> state_;
    };

    Topic() : core_(std::make_shared<Core>()) {}
    ~Topic() { Close(); }

    // 主题本身不可复制或移动：复制会导致多个主题共享订阅者注册表。
    Topic(const Topic&) = delete;
    Topic& operator=(const Topic&) = delete;
    Topic(Topic&&) = delete;
    Topic& operator=(Topic&&) = delete;

    /// 创建一个拥有独立有界队列的订阅者。
    ///
    /// @param queue_capacity 队列容量（>0）。队列满时的行为由 overflow_policy 决定。
    /// @param overflow_policy 溢出策略，默认丢弃最旧消息（kDropOldest）。
    /// @return 订阅句柄，移动给实际消费线程使用。
    /// @throw std::invalid_argument queue_capacity 为 0 时抛出。
    /// @note 主题已关闭后调用仍会成功返回，但句柄立即处于关闭状态。
    [[nodiscard]] Subscription Subscribe(
        std::size_t queue_capacity = 1,
        OverflowPolicy policy = OverflowPolicy::kDropOldest) {
        if (queue_capacity == 0) {
            throw std::invalid_argument("Topic subscription capacity must be greater than zero");
        }

        auto state = std::make_shared<SubscriberState>(queue_capacity, policy);
        {
            std::lock_guard<std::mutex> lock(core_->mutex);
            if (core_->closed) {
                state->closed = true;
            } else {
                core_->subscribers.emplace_back(state);
            }
        }
        return Subscription(core_, std::move(state));
    }

    /// 发布已经由智能指针管理的不可变消息。
    ///
    /// 同一 MessagePtr 会分发到所有有效订阅者，不发生 Message 对象拷贝。
    /// 分发期间持锁时间短（仅入队），慢订阅者不会阻塞发布者。
    ///
    /// @param message 待发布的消息句柄。
    /// @return 发布统计结果。空指针或主题已关闭时 accepted 为 false。
    [[nodiscard]] PublishResult Publish(MessagePtr message) {
        PublishResult result;
        if (!message) {
            return result;
        }

        std::vector<std::shared_ptr<SubscriberState>> subscribers;
        {
            std::lock_guard<std::mutex> lock(core_->mutex);
            if (core_->closed) {
                return result;
            }

            // 顺带清理已失效（订阅句柄析构）的订阅者条目。
            auto iterator = core_->subscribers.begin();
            while (iterator != core_->subscribers.end()) {
                if (auto subscriber = iterator->lock()) {
                    subscribers.emplace_back(std::move(subscriber));
                    ++iterator;
                } else {
                    iterator = core_->subscribers.erase(iterator);
                }
            }
        }

        result.accepted = true;
        result.subscriber_count = subscribers.size();

        for (const auto& subscriber : subscribers) {
            bool delivered = false;
            {
                std::lock_guard<std::mutex> lock(subscriber->mutex);
                if (subscriber->closed) {
                    continue;
                }

                // 队列已满时按策略处理：丢弃最旧或拒绝最新。
                if (subscriber->messages.size() >= subscriber->capacity) {
                    ++subscriber->dropped_count;
                    ++result.dropped_count;
                    if (subscriber->policy == OverflowPolicy::kRejectNewest) {
                        continue;
                    }
                    subscriber->messages.pop_front();
                }

                subscriber->messages.emplace_back(message);
                delivered = true;
                ++result.delivered_count;
            }
            if (delivered) {
                subscriber->condition.notify_one();
            }
        }

        return result;
    }

    /// 使用 std::make_shared 创建一次消息并发布，等价于
    /// Publish(std::make_shared<const Message>(args...))。
    /// @return 发布统计结果。
    template <typename... Args>
    [[nodiscard]] PublishResult Emplace(Args&&... args) {
        return Publish(std::make_shared<const Message>(std::forward<Args>(args)...));
    }

    /// 关闭主题并唤醒所有等待中的订阅者。
    /// 已入队消息仍可被取完；关闭后 Publish 返回 accepted=false，Subscribe 返回
    /// 已关闭句柄。析构函数会自动调用本方法，因此无需显式关闭（除非需要提前
    /// 停止某条链路）。
    void Close() noexcept {
        std::vector<std::shared_ptr<SubscriberState>> subscribers;
        {
            std::lock_guard<std::mutex> lock(core_->mutex);
            if (core_->closed) {
                return;
            }
            core_->closed = true;

            for (const auto& weak_subscriber : core_->subscribers) {
                if (auto subscriber = weak_subscriber.lock()) {
                    subscribers.emplace_back(std::move(subscriber));
                }
            }
            core_->subscribers.clear();
        }

        for (const auto& subscriber : subscribers) {
            {
                std::lock_guard<std::mutex> lock(subscriber->mutex);
                subscriber->closed = true;
            }
            subscriber->condition.notify_all();
        }
    }

    /// 主题是否已关闭。
    [[nodiscard]] bool IsClosed() const {
        std::lock_guard<std::mutex> lock(core_->mutex);
        return core_->closed;
    }

    /// 当前有效订阅者数量（自动清理已失效的订阅条目）。
    [[nodiscard]] std::size_t SubscriberCount() const {
        std::lock_guard<std::mutex> lock(core_->mutex);
        std::size_t count = 0;
        auto iterator = core_->subscribers.begin();
        while (iterator != core_->subscribers.end()) {
            if (iterator->expired()) {
                iterator = core_->subscribers.erase(iterator);
            } else {
                ++count;
                ++iterator;
            }
        }
        return count;
    }

private:
    std::shared_ptr<Core> core_;
};

}  // namespace drone::common

#endif  // DRONE_COMMON_TOPIC_H_
