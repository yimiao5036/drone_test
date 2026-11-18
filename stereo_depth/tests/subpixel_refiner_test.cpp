#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "stereo_depth/subpixel_refiner.h"

namespace sd = stereo_depth;

TEST(ParabolicSubpixel, SymmetricGivesZero) {
    sd::ParabolicSubpixel p;
    EXPECT_NEAR(p.Refine(10.0, 5.0, 10.0), 0.0, 1e-9);
}

TEST(ParabolicSubpixel, MatchesClosedForm) {
    sd::ParabolicSubpixel p;
    const double expected = 0.5 * (10.0 - 7.0) / (10.0 - 2 * 5.0 + 7.0);
    EXPECT_NEAR(p.Refine(10.0, 5.0, 7.0), expected, 1e-9);
}

TEST(LinearSubpixel, RecoversPositiveDelta) {
    // V 型模型 a=10,b=0,δ=0.3 → C-=13,C0=3,C+=7
    sd::LinearSubpixel l;
    EXPECT_NEAR(l.Refine(13.0, 3.0, 7.0), 0.3, 1e-6);
}

TEST(LinearSubpixel, RecoversNegativeDelta) {
    // δ=-0.3 → C-=7,C0=3,C+=13
    sd::LinearSubpixel l;
    EXPECT_NEAR(l.Refine(7.0, 3.0, 13.0), -0.3, 1e-6);
}

TEST(LinearSubpixel, LessBiasedThanParabolic) {
    sd::LinearSubpixel l;
    sd::ParabolicSubpixel p;
    const double truth = 0.3;
    const double dl = l.Refine(13.0, 3.0, 7.0);
    const double dp = p.Refine(13.0, 3.0, 7.0);
    EXPECT_LT(std::abs(dl - truth), std::abs(dp - truth));
}

TEST(Refiners, DegenerateReturnsZero) {
    sd::LinearSubpixel l;
    sd::ParabolicSubpixel p;
    // 无曲率（den<=0）
    EXPECT_NEAR(l.Refine(5.0, 5.0, 5.0), 0.0, 1e-9);
    EXPECT_NEAR(p.Refine(5.0, 5.0, 5.0), 0.0, 1e-9);
}

TEST(CreateSubpixelRefiner, NamesAndInvalid) {
    std::unique_ptr<sd::ISubpixelRefiner> a(sd::CreateSubpixelRefiner("linear"));
    std::unique_ptr<sd::ISubpixelRefiner> b(sd::CreateSubpixelRefiner("parabolic"));
    EXPECT_NE(a, nullptr);
    EXPECT_NE(b, nullptr);
    EXPECT_THROW(sd::CreateSubpixelRefiner("bogus"), std::invalid_argument);
}
