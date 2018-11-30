#include "gtest/gtest.h"

#include "ObstaclePredictor.h"

TEST(PredictionTests, OneRight)
{
	auto Predictor = ObstaclePredictor(0.05, 45, 55, 0.1);
	Predictor.AddVehicleRight(20, 10);
	auto constraints = Predictor.ProvideConstraints();
	EXPECT_LE(constraints[0].first,2.5);
	EXPECT_GE(constraints[0].second,3.5);
}

TEST(PredictionTests, TwoRight1)
{
	auto Predictor = ObstaclePredictor(0.05, 45, 55, 0.1);
	Predictor.AddVehicleRight(20, 10);
	Predictor.AddVehicleRight(10, 15);
	auto constraints = Predictor.ProvideConstraints();
	EXPECT_LE(constraints[0].first,2.5);
	EXPECT_GE(constraints[0].second,3.5);
	EXPECT_EQ(constraints[0].second,constraints[1].first);
}

TEST(PredictionTests, TwoRight2)
{
	auto Predictor = ObstaclePredictor(0.05, 45, 55, 0.1);
	Predictor.AddVehicleRight(20, 15);
	Predictor.AddVehicleRight(10, 10);
	auto constraints = Predictor.ProvideConstraints();
	EXPECT_LE(constraints[1].first,3.5);
	EXPECT_GE(constraints[1].second,4.5);
}

TEST(PredictionTests, TwoRightOneOpposite)
{
	auto Predictor = ObstaclePredictor(0.05, 45, 55, 0.1);
	Predictor.AddVehicleRight(20, 15);
	Predictor.AddVehicleRight(10, 10);
	Predictor.AddVehicleOpposite(10, 10);
	auto constraints = Predictor.ProvideConstraints();
	EXPECT_GE(constraints[1].second,6.5);
}


