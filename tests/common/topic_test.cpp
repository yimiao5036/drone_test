#include "common/topic.h"

#include <chrono>
#include <memory>
#include <stdexcept>

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

}  // namespace
}  // namespace drone::common
