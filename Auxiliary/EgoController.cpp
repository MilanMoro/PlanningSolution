#include "EgoController.h"

void EgoController::Reset()
{
	SetState(0,0,0);
	ResetLimits();
	Predictor.DeleteCars();
	nTimeStepElapsed = 0;
	isCarOnLane = false;
	actualTrajectory.clear();
}

void EgoController::SetState(double s, double v)
{
	Planner.state= {s,v,(v-Planner.state.v)/ts};
}

void EgoController::SetState(double s, double v, double a)
{
	Planner.state= {s,v,a};
}

void EgoController::AddCar(int lane, double posX, double posY, double vX, double vY)
{
	//Add car to the same lane if ahead
	if ((lane == 0) && ((posX+50)>Planner.state.s) && (posX<30))
	{
		//More than 1 additional car in the lane 0 in not handled currently
		isCarOnLane = true;
		carOnLaneSpeed = vX;
	}
	else if((lane == 1) && (vY>=0) && (posY < 4))
	{
		Predictor.AddVehicleRight(posY+50, vY);
	}
	else if((lane == 2) && (vX<=0) && (posX > -4))
	{
		Predictor.AddVehicleOpposite(-posX+50, -vX);
	}
	else if((lane == 3) && (vY<=0) && (posY > -4))
	{
		Predictor.AddVehicleLeft(-posY+50, vY);
	}
}

void EgoController::PlanRoute()
{
	nTimeStepElapsed = 0;
	SetConstraints();
	if (isCarOnLane && (carOnLaneSpeed < Planner.state.v))
	{
		// If there is a car ahead with higher speed, decrease speed to avoid collison
		actualTrajectory = Planner.CalculateTrajectoryToSpeed(carOnLaneSpeed);
	}
	else if (isCarOnLane)
	{
		// We estimate that the car ahead will not change its speed during the course of the crossing
		// To surely avoid collison we do not allow the ego car to increase its speed above the car's ahead
		if ((carOnLaneSpeed < originalLimits.v_max) && (carOnLaneSpeed != 0))
		{
			Planner.limits.v_max = carOnLaneSpeed;
		}
		if (carOnLaneSpeed != 0)
		{
			actualTrajectory  = Planner.CalculateOptimalTrajectory();
		}
		else
		{
			actualTrajectory = Planner.CalculateTrajectoryToSpeed(0);
		}
	}
	else
	{
		//If there is no car ahead calculate optimal trajectory
		actualTrajectory  = Planner.CalculateOptimalTrajectory();
	}
	if (actualTrajectory.empty())
	{
		// If the planner can not return any trajectory, and the ego car is not in the crossing decrease speed to 0
		if (Planner.state.s < crossingStart || Planner.state.s > crossingEnd)
		{
			actualTrajectory = Planner.CalculateTrajectoryToSpeed(0);
		}
		else
		{
			actualTrajectory  = {Planner.state.v};
		}
	}
}

double EgoController::ActualVelocityControlValue()
{
	double velocityControl;

	//If we run out of the planned trajectory, replan
	if (int(actualTrajectory.size()) < nTimeStepElapsed)
	{
		PlanRoute();
	}
	//If auto replanning is active raplan
	else if (autoReplanning && (nTimeStepElapsed >= nTimeStepToReplan))
	{
		PlanRoute();
	}
	if(!actualTrajectory.empty())
	{
		velocityControl = actualTrajectory[nTimeStepElapsed];
	}
	else
	{
		// If all trajectory calculation fail (even emergency brake), keep speed
		velocityControl = Planner.state.v;
	}
	//Prepare Controller for the next time step
	StepTime();
	return velocityControl;
}

void EgoController::StepTime()
{
	nTimeStepElapsed += 1;
	if (!isCarOnLane)
	{
		ResetLimits();
	}
	// we put the car back in the next time step if till present
	isCarOnLane = false;
	Predictor.DeleteCars();
}

void EgoController::SetConstraints()
{
	Planner.SetConstraints(Predictor.ProvideConstraints());
}

void EgoController::ResetLimits()
{
	Planner.limits = originalLimits;
}
