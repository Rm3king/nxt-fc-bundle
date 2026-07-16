/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "IndiTakeoverGate.hpp"

#include <gtest/gtest.h>

TEST(IndiTakeoverGate, RequiresContinuousAirborneDwell)
{
	IndiTakeoverGate gate;
	gate.update(1'000, false, false, false);
	EXPECT_FALSE(gate.ready(1'000 + IndiTakeoverGate::kAirborneDwellUs));
	EXPECT_TRUE(gate.ready(1'001 + IndiTakeoverGate::kAirborneDwellUs));
}

TEST(IndiTakeoverGate, EachContactStateClosesGate)
{
	IndiTakeoverGate gate;

	gate.update(1'000, false, false, false);
	ASSERT_TRUE(gate.ready(1'001 + IndiTakeoverGate::kAirborneDwellUs));
	gate.update(2'000'000, true, false, false);
	EXPECT_FALSE(gate.ready(3'000'001));

	gate.update(3'100'000, false, false, false);
	ASSERT_TRUE(gate.ready(4'100'001));
	gate.update(4'200'000, false, true, false);
	EXPECT_FALSE(gate.ready(5'200'001));

	gate.update(5'300'000, false, false, false);
	ASSERT_TRUE(gate.ready(6'300'001));
	gate.update(6'400'000, false, false, true);
	EXPECT_FALSE(gate.ready(7'400'001));
}

TEST(IndiTakeoverGate, ResetRequiresNewAirborneObservation)
{
	IndiTakeoverGate gate;
	gate.update(1'000, false, false, false);
	ASSERT_TRUE(gate.ready(1'001 + IndiTakeoverGate::kAirborneDwellUs));
	gate.reset();
	EXPECT_FALSE(gate.ready(3'000'000));
	gate.update(3'000'000, false, false, false);
	EXPECT_TRUE(gate.ready(4'000'001));
}
