#ifndef EGOCONTROLLER_H_
#define EGOCONTROLLER_H_

#include "ObstaclePredictor.h"
#include "RoutePlanner.h"

class EgoController {
public:
	EgoController(VehicleDynamicLimits limits, double timeStep, double crossStart, double crossEnd, double uncertanity)
			:Planner({limits},timeStep,crossStart,crossEnd), Predictor(uncertanity,crossStart,crossEnd,timeStep)
			{originalLimits = limits; ts = timeStep; crossingEnd = crossEnd; crossingStart = crossStart;};

	void Reset();
	void SetState(double s, double v);
	void SetState(double s, double v, double a);

	void AddCar(int lane, double posX, double posY, double vX, double vY);

	void PlanRoute();
	double ActualVelocityControlValue();
	void EnableAutoReplanning(int nTimeStep){autoReplanning = true;nTimeStepToReplan = nTimeStep;};
	void DisableAutoReplanning(){autoReplanning = false;};

private:
	bool autoReplanning = false;
	int nTimeStepToReplan=1;
	int nTimeStepElapsed = 0;
	bool isCarOnLane = false;
	double carOnLaneSpeed = 0;
	double crossingEnd;
	double crossingStart;
	double ts;
	std::vector<double> actualTrajectory;
	RoutePlanner Planner;
	ObstaclePredictor Predictor;
	VehicleDynamicLimits originalLimits;

	void SetConstraints();
	void ResetLimits();
	void StepTime();
};

#endif /* EGOCONTROLLER_H_ */
