#ifndef ROUTEPLANNER_H_
#define ROUTEPLANNER_H_

#include <vector>

#define MAX_COST 1000000

struct VehicleDynamicLimits
{
	double v_max;
	double a_max;
	double j_max;
};

struct State
{
	double s;
	double v;
	double a;
};

struct TrajectoryWithCost
{
	std::vector<double> trajectory;
	double cost;
};

class RoutePlanner
{
public:
	//TODO: Getters, Setters...
	RoutePlanner(VehicleDynamicLimits l, double t, double crossStart, double crossEnd) : limits(l), ts(t), crossingStart(crossStart), crossingEnd(crossEnd){};
	virtual ~RoutePlanner(){};
	VehicleDynamicLimits limits;

	State state = {0,0,0};
	double ts = 0.1;
	double sGoal = 80;

	std::vector<double> CalculateTrajectoryToSpeed(double vEnd);
	std::vector<double> CalculateOptimalTrajectory();

	void AddConstraint(std::pair<double,double>);
	void SetConstraints(std::vector<std::pair<double,double>>);
	void ModifyConstraint(unsigned nSequence , std::pair<double,double>);
	void DeleteConstraint(unsigned nSequence);

private:
	double crossingStart = 30;
	double crossingEnd = 40;
	// firstTypeConstraints - Given Position (crossingEnd) must be reached in provided time
	// secondTypeConstraints - Given Position (crossingStart) must be reached after provided time
	std::vector<std::pair<double,double>> Constraints;

	double CostFromATrajectory(std::vector<double>);

	std::vector<double> CalculateATrajectory(double vStart, double vEnd, double aStart);
	double CalculateSpeedOnAccelerationChange(double aStart, double aEnd);

	std::vector<double> CalculateFreeRoadATrajectory(double vMax);
	std::vector<double> CalculateCrossAheadATrajectory(double vMax);
	std::vector<double> CalculateCrossBehindATrajectory(double v1, double t, double v2);

	TrajectoryWithCost CalculateOptimalFreeRoadTrajectory();
	TrajectoryWithCost CalculateOptimalCrossingAheadFirstTrajectory();
	TrajectoryWithCost CalculateOptimalCrossingBetweenTrajectory(int nCar);//Between nCar and nCar+1
	TrajectoryWithCost CalculateOptimalCrossingBehindLastTrajectory();

	double CostCrossAhead(std::vector<double> arg,int nCar); //if nCar < 0 FreeRoad Cost is calculated
	double CostCrossBehind(std::vector<double> arg,int nCar);
	double CostCrossBetween(std::vector<double> arg,int nCar); //Between nCar and nCar+1
	double MinTimeToReach(double position);
	std::vector<double> FixSpeedLimitRoundingError(std::vector<double> vTrajectory);
};
#endif /* ROUTEPLANNER_H_ */
