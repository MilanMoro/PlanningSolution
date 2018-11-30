#include "gtest/gtest.h"

#include "Auxiliary.h"

#define private public
#include "EgoController.h"

TEST(ControllerTests, SimpleAheadTest)
{
	std::vector<double> resultTrajectory;
	auto Control = EgoController({20,1.999,1.999}, 0.1, 45, 55, 0.05);
	Control.SetState(10,18,0);
	double vOther = 10;
	double posOther = 10;
	double v_control;
	Control.EnableAutoReplanning(0);
	while(Control.Planner.state.s < 80)
	{
		if (posOther<55)
		{
			Control.Predictor.AddVehicleRight(posOther,vOther);
		}
		v_control = Control.ActualVelocityControlValue();
		resultTrajectory.push_back(v_control);

		Control.SetState(Control.Planner.state.s+0.1*v_control,v_control);
		posOther = posOther + 0.1* vOther;
	}
	auto sTrajectory = DiscreteIntegrator(resultTrajectory, 10, 0.1);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),(55)));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*0.1;
	EXPECT_LE(time,3.1);
}

TEST(ControllerTests, SimpleBehindTest)
{
	std::vector<double> resultTrajectory;
	auto Control = EgoController({20,2,1.999}, 0.1, 45, 55, 0.05);
	Control.SetState(10,10,0);
	double vOther = 10;
	double posOther = 10;
	double v_control;
	Control.EnableAutoReplanning(0);
	while(Control.Planner.state.s < 80)
	{
		if (posOther<55)
		{
			Control.Predictor.AddVehicleRight(posOther,vOther);
		}
		v_control = Control.ActualVelocityControlValue();
		resultTrajectory.push_back(v_control);

		Control.SetState(Control.Planner.state.s+0.1*v_control,v_control);
		posOther = posOther + 0.1* vOther;
	}
	auto sTrajectory = DiscreteIntegrator(resultTrajectory, 10, 0.1);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),(45)));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*0.1;
	EXPECT_GE(time,4.1);
}

TEST(ControllerTests, OtherStopTest)
{
	std::vector<double> resultTrajectory;
	auto Control = EgoController({20,2,1.999}, 0.1, 45, 55, 0.05);
	Control.SetState(10,10,0);
	double vOther = 10;
	double posOther = 10;
	double v_control;
	Control.EnableAutoReplanning(0);
	while(Control.Planner.state.s < 80)
	{
		bool errorFlag = false;
		if (posOther<55)
		{
			Control.Predictor.AddVehicleRight(posOther,vOther);
		}
		v_control = Control.ActualVelocityControlValue();
		resultTrajectory.push_back(v_control);
		Control.SetState(Control.Planner.state.s+0.1*v_control,v_control);
		posOther = posOther + 0.1* vOther;

		if (posOther > 40)
		{
			vOther = 0;
			errorFlag = true;
		}
		if (errorFlag)
		{
			vOther = 5;
		}
	}
	auto sTrajectory = DiscreteIntegrator(resultTrajectory, 10, 0.1);
	auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),(45)));
	double time = (std::distance(sTrajectory.begin(),pos)+1)*0.1;
	EXPECT_GE(time,4.1);
}




