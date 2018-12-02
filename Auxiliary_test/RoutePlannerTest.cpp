#include <math.h>
#include <stdlib.h>
#include <numeric>
#include "gtest/gtest.h"

#define private public
// TODO create more classes and test only on public interfaces
#include "RoutePlanner.h"
#include "Auxiliary.h"

TEST(VehicleDynamicTests, AccelerationIncrease)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	double v = Planner.CalculateSpeedOnAccelerationChange(0, 2);
	EXPECT_LE(abs(v-1.3),0.01);
}

TEST(VehicleDynamicTests, AccelerationDecrease1)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	double v = Planner.CalculateSpeedOnAccelerationChange(2, 0);
	EXPECT_LE(abs(v-0.9),0.01);
}

TEST(VehicleDynamicTests, AccelerationDecrease2)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	double v = Planner.CalculateSpeedOnAccelerationChange(2, -2);
	EXPECT_LE(abs(v+0.4),0.01);
}

TEST(VehicleDynamicTests, MinTimeMaxSpeedReached)
{

	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {20, 17, 0};
	double t = Planner.MinTimeToReach(80);
	EXPECT_LE(abs(t-3.5),0.00001);
}

TEST(VehicleDynamicTests, MinTimeMaxSpeedNotReached)
{

	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {20, 15, 0};
	double t = Planner.MinTimeToReach(60);
	EXPECT_LE(abs(t-2.4),0.00001);
}

TEST(VehicleDynamicTests, SetSpeedIncreaseWithoutMaxAcc1)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(10, 11.5, 0);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(1.5-sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = (*max_element(jVec.begin(), jVec.end()))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedIncreaseWithoutMaxAcc2)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(10, 11.5, 1);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(1.5-sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = (*max_element(std::next(jVec.begin(),1), jVec.end()))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedIncreaseWithMaxAcc1)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(10, 20, 0);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(10-sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = (*max_element(std::next(jVec.begin(),1), jVec.end()))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedIncreaseWithMaxAcc2)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(10, 20, -0.5);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(10-sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = (*max_element(std::next(jVec.begin(),1), jVec.end()))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedDecreaseWithoutMaxAcc1)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(11.5, 10, 0);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(1.5+sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = abs((*min_element(std::next(jVec.begin(),1), jVec.end())))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedDecreaseWithoutMaxAcc2)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(11.5, 10, -1);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(1.5+sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = abs((*min_element(std::next(jVec.begin(),1), jVec.end())))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedDecreaseWithMaxAcc1)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(20, 10, 0);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(10+sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = abs((*min_element(std::next(jVec.begin(),1), jVec.end())))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedDecreaseWithMaxAcc2)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(20, 10, -0.5);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_LE(abs(10+sumA*0.1),0.001);
	//Test if jerk limit is violated
	std::vector<double> jVec = aTraj;
	std::adjacent_difference(aTraj.begin(), aTraj.end(),jVec.begin());
	double maxJerk = abs((*min_element(std::next(jVec.begin(),1), jVec.end())))*10;
	EXPECT_LT(maxJerk,2);
}

TEST(VehicleDynamicTests, SetSpeedConstant)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	std::vector<double> aTraj = Planner.CalculateATrajectory(10, 10, 0);
	//Test if the sum of accelerations equals the velocity change
	double sumA = std::accumulate(aTraj.begin(), aTraj.end(), 0.0);
	EXPECT_EQ(sumA,0);
}

TEST(OptimizationTests, FreeRoad)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 5, 0};
	Planner.sGoal = 80;
	TrajectoryWithCost result = Planner.CalculateOptimalFreeRoadTrajectory();
	EXPECT_LE(result.cost,ceil(10*Planner.sGoal/Planner.state.v)/10+0.0001);
}

TEST(OptimizationTests, FreeRoadNoMaxSpeed)
{
	auto Planner = RoutePlanner({50,2,1.999},0.1,35,40);
	Planner.state = {0, 5, 0};
	Planner.sGoal = 80;
	TrajectoryWithCost result = Planner.CalculateOptimalFreeRoadTrajectory();
	EXPECT_LE(result.cost,ceil(10*Planner.sGoal/Planner.state.v)/10+0.0001);
}

TEST(OptimizationTests, CrossAheadSuccessTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 5, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({5,20});
	TrajectoryWithCost result1 = Planner.CalculateOptimalCrossingAheadFirstTrajectory();
	EXPECT_GE(result1.trajectory.size(),1);
	Planner.ModifyConstraint(0,{6,20});
	TrajectoryWithCost result2 = Planner.CalculateOptimalCrossingAheadFirstTrajectory();
	EXPECT_GE(result2.trajectory.size(),1);
	EXPECT_LE(result2.cost,result1.cost);
}

TEST(OptimizationTests, CrossAheadImpossibleTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 5, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({4,10});
	TrajectoryWithCost result = Planner.CalculateOptimalCrossingAheadFirstTrajectory();
	EXPECT_EQ(result.cost,MAX_COST);
}

TEST(OptimizationTests, CrossBehindSuccessTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,50,60);
	Planner.state = {0, 10, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({5,8});
	TrajectoryWithCost result1 = Planner.CalculateOptimalCrossingBehindLastTrajectory();
	EXPECT_GE(result1.trajectory.size(),1);
	Planner.ModifyConstraint(0,{5,12});
	TrajectoryWithCost result2 = Planner.CalculateOptimalCrossingBehindLastTrajectory();
	EXPECT_GE(result2.trajectory.size(),1);
	EXPECT_LE(result1.cost,result2.cost);
}

TEST(OptimizationTests, CrossBehindBugTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,46,58);
	Planner.state = {50-7.89165,0,0};
	Planner.sGoal = 80;
	Planner.AddConstraint({0,2.96095});
	auto result1 = Planner.CalculateCrossBehindATrajectory(0,0,5.9565);
}

TEST(OptimizationTests, CrossBehindLowSpeedTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 1, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({5,8});
	TrajectoryWithCost result1 = Planner.CalculateOptimalCrossingBehindLastTrajectory();
	EXPECT_GE(result1.trajectory.size(),1);
}

TEST(OptimizationTests, CrossBehindImpossibleTest)
{
	//Not possible to stop before the crossing
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 15, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({4,30});
	TrajectoryWithCost result = Planner.CalculateOptimalCrossingBehindLastTrajectory();
	EXPECT_EQ(result.cost,MAX_COST);
}

TEST(OptimizationTests, CrossBetweenSuccessTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,45,55);
	Planner.state = {0, 5, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({5,8});
	Planner.AddConstraint({12,15});
	Planner.AddConstraint({18,20});
	TrajectoryWithCost result1 = Planner.CalculateOptimalCrossingBetweenTrajectory(0);
	EXPECT_GE(result1.trajectory.size(),1);
	TrajectoryWithCost result2 = Planner.CalculateOptimalCrossingBetweenTrajectory(1);
	EXPECT_GE(result2.trajectory.size(),1);
	EXPECT_LE(result1.cost,result2.cost);
}

TEST(OptimizationTests, CrossBetweenImpossibleTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,35,40);
	Planner.state = {0, 15, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({4,10});
	Planner.AddConstraint({8,15});
	TrajectoryWithCost result = Planner.CalculateOptimalCrossingBetweenTrajectory(0);
	EXPECT_EQ(result.cost,MAX_COST);
}

TEST(OptimizationTests, OptimalTrajectoryTest1)
{
	//Optimum is crossing ahead of first
	auto Planner = RoutePlanner({20,2,1.999},0.1,45,55);
	Planner.state = {10, 15, -1};
	Planner.sGoal = 80;
	Planner.AddConstraint({5,8});
	Planner.AddConstraint({12,15});
	Planner.AddConstraint({18,20});
	auto vTrajectory = Planner.CalculateOptimalTrajectory();
	auto sTrajectory = DiscreteIntegrator(vTrajectory, Planner.state.s, Planner.ts);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),
			(Planner.crossingStart + Planner.crossingEnd)/2));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*Planner.ts;
	EXPECT_LE(time,5);
}

TEST(OptimizationTests, OptimalTrajectoryTest2)
{
	//Optimum is crossing between 2nd and 3rd car
	auto Planner = RoutePlanner({20,2,1.999},0.1,45,55);
	Planner.state = {10, 5, 1};
	Planner.sGoal = 80;
	Planner.AddConstraint({3,12});
	Planner.AddConstraint({12,15});
	Planner.AddConstraint({18,20});
	auto vTrajectory = Planner.CalculateOptimalTrajectory();
	auto sTrajectory = DiscreteIntegrator(vTrajectory, Planner.state.s, Planner.ts);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),
			(Planner.crossingStart + Planner.crossingEnd)/2));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*Planner.ts;
	EXPECT_TRUE((time >= 15) && (time <= 18));
}

TEST(OptimizationTests, OptimalTrajectoryTest3)
{
	//Optimum is crossing after the last car
	auto Planner = RoutePlanner({20,2,1.999},0.1,45,55);
	Planner.state = {10, 5, 0};
	Planner.sGoal = 80;
	Planner.AddConstraint({1,12});
	Planner.AddConstraint({12,17.5});
	Planner.AddConstraint({18,20});
	auto vTrajectory = Planner.CalculateOptimalTrajectory();
	auto sTrajectory = DiscreteIntegrator(vTrajectory, Planner.state.s, Planner.ts);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),
			(Planner.crossingStart + Planner.crossingEnd)/2));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*Planner.ts;
	EXPECT_GE(time,20);
}

TEST(OptimizationTests, OptimalTrajectoryNoRouteTest)
{
	auto Planner = RoutePlanner({20,2,1.999},0.1,45,55);
	Planner.state = {30, 15, -1};
	Planner.sGoal = 80;
	Planner.AddConstraint({1,8});
	Planner.AddConstraint({12,15});
	Planner.AddConstraint({18,20});
	auto result = Planner.CalculateOptimalTrajectory();
	EXPECT_EQ(result.size(),0);
}
