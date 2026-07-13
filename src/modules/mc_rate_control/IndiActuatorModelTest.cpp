/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "IndiActuatorModel.hpp"

#include <gtest/gtest.h>

using matrix::Vector3f;

TEST(IndiActuatorModel, TimestampInterpolation)
{
	IndiActuatorModel model;
	model.push(1000, Vector3f{0.f, 1.f, 2.f});
	model.push(2000, Vector3f{1.f, 2.f, 3.f});
	Vector3f sample{};
	ASSERT_TRUE(model.sample(1500, sample));
	EXPECT_NEAR(sample(0), 0.5f, 1e-6f);
	EXPECT_NEAR(sample(1), 1.5f, 1e-6f);
	EXPECT_NEAR(sample(2), 2.5f, 1e-6f);
}

TEST(IndiActuatorModel, FirstOrderStep)
{
	IndiActuatorModel model;
	model.push(1000, Vector3f{});
	Vector3f state{};
	ASSERT_TRUE(model.update(1000, 0.01f, 0.f, 0.1f, state));
	model.push(2000, Vector3f{1.f, 1.f, 1.f});

	for (int i = 1; i <= 100; ++i) {
		model.push(2000 + i * 1000, Vector3f{1.f, 1.f, 1.f});
		ASSERT_TRUE(model.update(2000 + i * 1000, 0.01f, 0.f, 0.1f, state));
	}

	EXPECT_NEAR(state(0), 1.f - expf(-10.f), 1e-4f);
}

TEST(IndiActuatorModel, ResetRejectsMissingHistory)
{
	IndiActuatorModel model;
	Vector3f state{};
	EXPECT_FALSE(model.update(1000, 0.001f, 0.f, 0.04f, state));
	model.push(1000, Vector3f{0.2f, 0.3f, 0.4f});
	EXPECT_TRUE(model.update(1000, 0.001f, 0.f, 0.04f, state));
	model.reset();
	EXPECT_FALSE(model.update(2000, 0.001f, 0.f, 0.04f, state));
}

TEST(IndiActuatorModel, RejectsUnavailableDelayHistory)
{
	IndiActuatorModel model;
	model.push(1000, Vector3f{0.2f, 0.3f, 0.4f});
	model.push(2000, Vector3f{0.3f, 0.4f, 0.5f});
	Vector3f state{};

	EXPECT_FALSE(model.update(2000, 0.001f, 0.0015f, 0.04f, state));
	EXPECT_TRUE(model.update(2000, 0.001f, 0.001f, 0.04f, state));
}
