#include "common/topic.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

namespace drone::common {
namespace {

using namespace std::chrono_literals;

TEST(TopicTest, DeliversSameMessageInstanceToEverySubscriber) {
    Topic<int> topic;
    auto first = topic.Subscribe(2);
    auto second = topic.Subscribe(2);
    auto message = std::make_shared<const int>(42);

    const auto publish_result = topic.Publish(message);

    ASSERT_TRUE(publish_result.accepted);
    EXPECT_EQ(publish_result.subscriber_count, 2U);
    EXPECT_EQ(publish_result.delivered_count, 2U);
    ASSERT_TRUE(first.TryTake().has_value());
    const auto second_message = second.TryTake();
    ASSERT_TRUE(second_message.has_value());
    EXPECT_EQ(second_message.value().get(), message.get());
    EXPECT_EQ(*second_message.value(), 42);
}

TEST(TopicTest, DropsOldestMessageForRealtimeSubscriber) {
    Topic<int> topic;
    auto subscription = topic.Subscribe(2, Topic<int>::OverflowPolicy::kDropOldest);

    EXPECT_TRUE(topic.Emplace(1).accepted);
    EXPECT_TRUE(topic.Emplace(2).accepted);
    const auto result = topic.Emplace(3);

    EXPECT_EQ(result.dropped_count, 1U);
    EXPECT_EQ(subscription.DroppedCount(), 1U);
    const auto first = subscription.TryTake();
    const auto second = subscription.TryTake();
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(**first, 2);
    EXPECT_EQ(**second, 3);
}

TEST(TopicTest, RejectsNewestMessageWhenConfigured) {
    Topic<int> topic;
    auto subscription = topic.Subscribe(1, Topic<int>::OverflowPolicy::kRejectNewest);

    EXPECT_TRUE(topic.Emplace(10).accepted);
    const auto result = topic.Emplace(20);

    EXPECT_EQ(result.delivered_count, 0U);
    EXPECT_EQ(result.dropped_count, 1U);
    const auto message = subscription.TryTake();
    ASSERT_TRUE(message.has_value());
    EXPECT_EQ(**message, 10);
}

TEST(TopicTest, CloseWakesSubscriberAndRejectsNewMessages) {
    Topic<int> topic;
    auto subscription = topic.Subscribe();

    topic.Close();

    EXPECT_TRUE(topic.IsClosed());
    EXPECT_FALSE(subscription.IsOpen());
    EXPECT_FALSE(subscription.WaitTakeFor(1ms).has_value());
    EXPECT_FALSE(topic.Emplace(7).accepted);
}

TEST(TopicTest, SubscriptionUsesRaIIToUnsubscribe) {
    Topic<int> topic;
    {
        auto subscription = topic.Subscribe();
        EXPECT_EQ(topic.SubscriberCount(), 1U);
    }

    EXPECT_EQ(topic.SubscriberCount(), 0U);
}

TEST(TopicTest, RejectsZeroCapacitySubscription) {
    Topic<int> topic;
    EXPECT_THROW(topic.Subscribe(0), std::invalid_argument);
}

// 停机路径回归：Reset() 常在线程 join 之前调用以唤醒阻塞的消费者（见各模块
// Stop()），历史上它与 TryTake/WaitTakeFor 并发访问 Subscription::state_ 构成
// data race。本用例让消费线程持续取消息、主线程并发 Reset，要求不崩溃且 Reset
// 后句柄失效；建议在 TSan/ASan 构建中运行以覆盖该竞争。
TEST(TopicTest, ResetIsSafeWhileAnotherThreadConsumes) {
    Topic<int> topic;
    auto subscription = topic.Subscribe(4);

    std::atomic<bool> stop{false};
    std::atomic<int> taken{0};
    std::thread consumer([&subscription, &stop, &taken] {
        while (!stop.load(std::memory_order_relaxed)) {
            if (subscription.TryTake().has_value()) {
                taken.fetch_add(1, std::memory_order_relaxed);
            }
            (void)subscription.WaitTakeFor(1ms);
        }
    });

    for (int i = 0; i < 2000; ++i) {
        (void)topic.Emplace(i);
    }
    subscription.Reset();  // 与消费线程的 TryTake/WaitTakeFor 并发
    stop.store(true, std::memory_order_relaxed);
    consumer.join();

    EXPECT_FALSE(subscription.IsOpen());
    EXPECT_FALSE(subscription.TryTake().has_value());
    EXPECT_EQ(topic.SubscriberCount(), 0U);
    EXPECT_GE(taken.load(std::memory_order_relaxed), 0);
}

}  // namespace
}  // namespace drone::common
