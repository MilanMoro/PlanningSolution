#include "RoutePlanner.h"
#include "Auxiliary.h"

#include <stdlib.h>
#include <limits>
#include <math.h>
#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <valarray>
#include <map>

std::vector<double> RoutePlanner::CalculateOptimalTrajectory()
{

	std::map<double,std::vector<double>> trajectoryMap;
	if((Constraints.size() == 0) || (state.s > crossingStart))
	{
		// If there are no constraints (no car on the right) or the ego car already left (or already in) the intersection
		return FixSpeedLimitRoundingError(CalculateOptimalFreeRoadTrajectory().trajectory);
	}

	// If there are constraints (one or more cars on the right)
	TrajectoryWithCost result;
	//Calculate optimal trajectory in case of passing ahead of the first car
	result = CalculateOptimalCrossingAheadFirstTrajectory();
	trajectoryMap.insert(std::pair<double,std::vector<double>>(result.cost, result.trajectory));

	//Calculate optimal trajectory in case of passing after of the last car
	result = CalculateOptimalCrossingBehindLastTrajectory();
	trajectoryMap.insert(std::pair<double,std::vector<double>>(result.cost, result.trajectory));

	for(int i = 0; i < int(Constraints.size())-1; i++)
	{
		//Calculate optimal trajectory passing between cars
		result = CalculateOptimalCrossingBetweenTrajectory(i);
		trajectoryMap.insert(std::pair<double,std::vector<double>>(result.cost, result.trajectory));
	}
	if (trajectoryMap.begin()->first != MAX_COST)
	{
		//If at least one trajectory found, return the one with the best cost
		return FixSpeedLimitRoundingError(trajectoryMap.begin()->second);
	}
	// If no trajectory found return an empty trajectory
	return std::vector<double>();
}

double RoutePlanner::CalculateSpeedOnAccelerationChange(double aStart,double aEnd)
{
	//This function calculates the minimal speed change during a change in acceleration
	double dv;
	double t = floor(abs((aEnd-aStart))/(limits.j_max*ts));
	if (aStart < aEnd)
		dv =  t*aStart*ts + (1+t)*(t/2)*limits.j_max*ts*ts + aEnd*ts;
	else
		dv =  t*aStart*ts - (1+t)*(t/2)*limits.j_max*ts*ts + aEnd*ts;
	return dv;
}

std::vector<double> RoutePlanner::CalculateTrajectoryToSpeed(double vEnd)
{
	return FixSpeedLimitRoundingError(DiscreteIntegrator(CalculateATrajectory(state.v,vEnd,state.a), state.v , ts));
}

TrajectoryWithCost RoutePlanner::CalculateOptimalFreeRoadTrajectory()
{
	std::vector<OptimizationArgumentOption> arg;
	// The Speed at the end of the run can not be lower than 1 m/s to avoid calculation really long trajectories
	arg.push_back({std::max(state.v,1.0),limits.v_max,0.1,std::max(state.v,1.0),true});
	OptimizationResult result =  OptimizeChemotaxis(this, 1000, arg, &RoutePlanner::CostCrossAhead, -1);
	double vMaxOptimal = result.optimalArguments[0];

	auto aTrajectory = CalculateFreeRoadATrajectory(vMaxOptimal);
	auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);
	return {vTrajectory, result.cost};
}

TrajectoryWithCost RoutePlanner::CalculateOptimalCrossingAheadFirstTrajectory()
{
	std::vector<double> vTrajectory;

	// Optimization would still yield the same result without this condition, but unnecessary in this case
	if (MinTimeToReach(crossingEnd) > Constraints[0].first)
	{
		return {vTrajectory, MAX_COST};
	}
	std::vector<OptimizationArgumentOption> arg;
	arg.push_back({std::max(state.v,1.0),limits.v_max,0.1,limits.v_max,true});
	OptimizationResult result =  OptimizeChemotaxis(this, 1000, arg, &RoutePlanner::CostCrossAhead, 0);
	double vMaxOptimal = result.optimalArguments[0];

	auto aTrajectory = CalculateCrossAheadATrajectory(vMaxOptimal);
	vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);
	if (result.cost == MAX_COST)
	{
		vTrajectory.clear();
	}
	return {vTrajectory, result.cost};
}

TrajectoryWithCost RoutePlanner::CalculateOptimalCrossingBehindLastTrajectory()
{
	std::vector<OptimizationArgumentOption> arg;
	arg.push_back({0,limits.v_max,0.1,0.5*(crossingStart-(state.s))/(Constraints.back().second+1),true}); //v1
	arg.push_back({0,std::min(Constraints.back().second,50.0),0.2,0.9*std::min(Constraints.back().second,50.0),true}); //t1
	arg.push_back({1,limits.v_max,0.1,limits.v_max/2,true}); //v2
	OptimizationResult result =  OptimizeChemotaxis(this, 2000, arg, &RoutePlanner::CostCrossBehind, int(Constraints.size())-1);
	double v1 = result.optimalArguments[0];
	double t1 = result.optimalArguments[1];
	double v2 = result.optimalArguments[2];

	auto aTrajectory = CalculateCrossBehindATrajectory(v1,t1,v2);
	auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);
	if (result.cost == MAX_COST)
	{
		vTrajectory.clear();
	}
	return {vTrajectory, result.cost};
}

TrajectoryWithCost RoutePlanner::CalculateOptimalCrossingBetweenTrajectory(int nCar)
{
	std::vector<OptimizationArgumentOption> arg;
	arg.push_back({0,limits.v_max,0.1,0.5*(crossingStart-(state.s))/(Constraints[nCar].second+1),true}); //v1
	arg.push_back({0,std::min(Constraints[nCar].second,50.0),0.25,0.9*std::min(Constraints[nCar].second,50.0),true}); //t1
	arg.push_back({1,limits.v_max,0.1,limits.v_max/2,true}); //v2
	OptimizationResult result =  OptimizeChemotaxis(this, 2000, arg, &RoutePlanner::CostCrossBetween, nCar);
	double v1 = result.optimalArguments[0];
	double t1 = result.optimalArguments[1];
	double v2 = result.optimalArguments[2];

	auto aTrajectory = CalculateCrossBehindATrajectory(v1,t1,v2);
	auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);
	if (result.cost == MAX_COST)
	{
		vTrajectory.clear();
	}
	return {vTrajectory, result.cost};
}

double RoutePlanner::CostCrossAhead(std::vector<double> arg, int nCar)
{
	/*The optimization target (arg) a scalar value, which represents the maximal speed */

	double vMax = arg[0];

	auto aTrajectory = CalculateCrossAheadATrajectory(vMax);

	if (nCar >= 0)
	{
		auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v, ts);
		auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),crossingEnd));
		// If the trajectory does not reach the end of the crossing in the given time
		double tCrossingEnd = (std::distance(sTrajectory.begin(),pos)+1)*ts;
		if (tCrossingEnd > Constraints[nCar].first)
		{
			return MAX_COST;
		}
	}
	return CostFromATrajectory(aTrajectory);
}

double RoutePlanner::CostCrossBehind(std::vector<double> arg, int nCar)
{
	/*The optimization targets are:
	 * v1 - First reachable speed
	 * t1 - Keep speed time
	 * v2 - Second reachable speed
	 */

	double v1 = arg[0];
	double t1 = arg[1];
	double v2 = arg[2];

	auto aTrajectory = CalculateCrossBehindATrajectory(v1,t1,v2);

	if (nCar >= 0)
	{
		//Constraint
		auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v, ts);
		auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),crossingStart));
		// If the trajectory  reaches the start of the crossing before the given time
		double tCrossingStart = (std::distance(sTrajectory.begin(),pos))*ts;
		if ((tCrossingStart < Constraints[nCar].second) || (pos ==sTrajectory.end()))
		{
			return MAX_COST;
		}
	}
	return CostFromATrajectory(aTrajectory);
}

double RoutePlanner::CostCrossBetween(std::vector<double> arg, int nCar)
{
	/*The optimization targets are:
	 * v1 - First reachable speed
	 * t1 - Keep speed time
	 * v2 - Second reachable speed
	 */

	double v1 = arg[0];
	double t1 = arg[1];
	double v2 = arg[2];

	auto aTrajectory = CalculateCrossBehindATrajectory(v1,t1,v2);

	if (nCar >= 0)
	{
		//Constraint
		auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v, ts);
		auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),crossingStart));
		double tCrossingStart = (std::distance(sTrajectory.begin(),pos))*ts;
		// If the trajectory  reaches the start of the crossing before the given time
		if (tCrossingStart < Constraints[nCar].second)
		{
			return MAX_COST;
		}
		pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),crossingEnd));
		// If the trajectory does not reach the end of the crossing in the given time
		double tCrossingEnd = (std::distance(sTrajectory.begin(),pos)+1)*ts;
		if (tCrossingEnd > Constraints[nCar+1].first)
		{
			return MAX_COST;
		}
	}
	return CostFromATrajectory(aTrajectory);
}

std::vector<double> RoutePlanner::CalculateFreeRoadATrajectory(double vMax)
{
	return CalculateCrossAheadATrajectory(vMax);
}

std::vector<double> RoutePlanner::CalculateCrossAheadATrajectory(double vMax)
{
	auto aTrajectory = CalculateATrajectory(state.v, vMax ,state.a);
	auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);

	double s1 = std::accumulate(vTrajectory.begin(), vTrajectory.end(), 0.0)*ts;
	if (s1 <= (sGoal-state.s))
	{
		double v;
		if (vTrajectory.empty())
		{
			v = state.v;
		}
		else
		{
			v = vTrajectory.back();
		}
		auto t2 = ceil(((sGoal-state.s)-s1)/v/ts);
		for(auto i = 0; i < t2; i++)
		{
			aTrajectory.push_back(0);
		}
	}
	else
	{
		auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),sGoal));
		aTrajectory.erase(aTrajectory.begin()+std::distance(sTrajectory.begin(),pos)+1,aTrajectory.end());
	}
	return aTrajectory;
}

std::vector<double> RoutePlanner::CalculateCrossBehindATrajectory(double v1, double t1,
		double v2)
{
	std::vector<double> aTrajectory;
	std::vector<double> vTrajectory;
	std::vector<double> sTrajectory;

	//Set v1
	aTrajectory = CalculateATrajectory(state.v, v1 ,state.a);
	vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);

	//Keep speed for t
	for(auto i = 0; i < floor(t1/ts); i++)
	{
		aTrajectory.push_back(0);
	}

	double v, a;
	if (vTrajectory.empty())
	{
		v = state.v;
		a = 0;
	}
	else
	{
		v = vTrajectory.back();
		a = aTrajectory.back();
	}

	//Set v2
	auto aTrajectoryTemp = CalculateATrajectory(v, v2 , a);
	aTrajectory.insert( aTrajectory.end(), aTrajectoryTemp.begin(), aTrajectoryTemp.end() );
	vTrajectory = DiscreteIntegrator(aTrajectory, state.v , ts);
	sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);

	//Keep Speed until the goal if not reached
	if (sTrajectory.back() <= sGoal)
	{
		auto t2 = ceil((sGoal-sTrajectory.back())/vTrajectory.back()/ts);
		for(auto i = 0; i < t2; i++)
		{
			vTrajectory.push_back(vTrajectory.back());
			aTrajectory.push_back(0);
		}
	}
	//If the goal is already reached, delete unnecessary trajectory
	else
	{
		auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),sGoal));
		if (pos != sTrajectory.end())
		{
			aTrajectory.erase(aTrajectory.begin()+std::distance(sTrajectory.begin(),pos)+1,aTrajectory.end());
		}
	}
	return aTrajectory;
}

double RoutePlanner::MinTimeToReach(double position)
{
	auto aTrajectory = CalculateATrajectory(state.v, limits.v_max, state.a);
	double t1 = aTrajectory.size()*ts;
	double s1;
	auto vTrajectory = DiscreteIntegrator(aTrajectory, state.v, ts);
	auto sTrajectory = DiscreteIntegrator(vTrajectory, state.s, ts);
	if (t1>0)
	{
		s1 = sTrajectory.back();
	}
	else
	{
		s1 = state.s;
	}
	if(s1<=position)
	{
		auto t2 = ceil((position-s1)/limits.v_max);
		//The cost is the total time + the summarized acceleration which is the total velocity change
		return t1+t2;
	}
	else
	{
		auto pos = std::find_if (sTrajectory.begin(), sTrajectory.end(),std::bind2nd(std::greater<double>(),position));
		sTrajectory.erase(std::next(pos,1),sTrajectory.end());
		return sTrajectory.size()*ts;
	}
}

void RoutePlanner::AddConstraint(std::pair<double,double> constraintPair)
{
	Constraints.push_back(constraintPair);
}

void RoutePlanner::SetConstraints(std::vector<std::pair<double, double> > constraints)
{
	Constraints = constraints;
}

void RoutePlanner::ModifyConstraint(unsigned nSequence, std::pair<double,double> constraintPair)
{
	if(Constraints.size()>nSequence)
	{
		Constraints[nSequence] = (constraintPair);
	}
}

void RoutePlanner::DeleteConstraint(unsigned nSequence)
{
	Constraints.erase(Constraints.begin() + nSequence);
}

double RoutePlanner::CostFromATrajectory(std::vector<double> aTrajectory)
{
	double sumA = 0;
	for (auto it = aTrajectory.begin(); it!=aTrajectory.end(); ++it)
	{
		sumA += abs(*it);
	}
	return aTrajectory.size()*ts + sumA*ts;
}
std::vector<double> RoutePlanner::FixSpeedLimitRoundingError(std::vector<double> vTrajectory)
{
	for(auto it = vTrajectory.begin(); it != vTrajectory.end(); ++it)
	{
		if (*it < 0)
		{
			*it = 0;
		}
		if (*it > limits.v_max)
		{
			*it = limits.v_max;
		}
	}
	return vTrajectory;
}

std::vector<double> RoutePlanner::CalculateATrajectory(double vStart, double vEnd, double aStart)
{
	//This function returns the applicable accelerations when changing speed from vStart to vEnd with minimal time and 0 aEnd
	//If vEnd is not reachable without an overshoot the function will ignore vEnd and reach a = 0 as fast as possible
	std::vector<double> aTraj;
	double a = aStart;
	double dv = 0;
	double dvGoal = vEnd-vStart;
	std::vector<double> aTrajTemp;
	if (dvGoal>=0)
	{
		double min_dv_Acc_Inc = CalculateSpeedOnAccelerationChange(aStart,limits.a_max);
		double min_dv_Acc_Dec = CalculateSpeedOnAccelerationChange(limits.a_max,0);
		// if a_max can not be reached
		if ((min_dv_Acc_Inc +  min_dv_Acc_Dec)> dvGoal)
		{
			//Jerk > 0
			while((dv + (a+limits.j_max*ts)*ts) < (dvGoal - CalculateSpeedOnAccelerationChange(a+limits.j_max*ts,0)))
			{
				a += limits.j_max*ts;
				dv += a*ts;
				aTraj.push_back(a);
			}
			if ((dv + a*ts) < (dvGoal - CalculateSpeedOnAccelerationChange(a,0)))
			{
				dv += a*ts;
				aTraj.push_back(a);
			}
			//Jerk < 0
			while((a-limits.j_max*ts) > 0.0001)
			{
				a -= limits.j_max*ts;
				dv += a*ts;
				aTrajTemp.push_back(a);
			}
			double aLast = (dvGoal - dv)/ts;
			if (aLast>0.0001)
			{
				aTrajTemp.push_back(aLast);
				//aLast can violate the jerk limit, so it is inserted into the right place in the vector
				std::sort (aTrajTemp.begin(), aTrajTemp.end(), std::greater<double>());
			}
			aTraj.insert(std::end(aTraj), std::begin(aTrajTemp), std::end(aTrajTemp));
		}
		//if A_max can be reached
		else
		{
			//Jerk > 0
			while((a+limits.j_max*ts) < limits.a_max)
			{
				a += limits.j_max*ts;
				aTraj.push_back(a);
				dv += a*ts;
			}
			//a = max
			a = limits.a_max;
			while((dv + min_dv_Acc_Dec+a*ts) <= dvGoal)
			{
				dv += a*ts;
				aTraj.push_back(a);
			}
			//Jerk < 0
			while((a-limits.j_max*ts) > 0.0001)
			{
				a -= limits.j_max*ts;
				dv += a*ts;
				aTrajTemp.push_back(a);
			}
			double aLast = (dvGoal - dv)/ts;
			if (aLast>0.0001)
			{
				aTrajTemp.push_back(aLast);
				//aLast can violate the jerk limit, so it is inserted into the right place in the vector
				std::sort (aTrajTemp.begin(), aTrajTemp.end(), std::greater<double>());
			}
			aTraj.insert(std::end(aTraj), std::begin(aTrajTemp), std::end(aTrajTemp));
		}
	}
	//if dvGoal < 0
	else
	{
		double min_dv_Acc_Dec = CalculateSpeedOnAccelerationChange(aStart,-limits.a_max);
		double min_dv_Acc_Inc = CalculateSpeedOnAccelerationChange(-limits.a_max,0);
		// if a_max can not be reached
		if ((min_dv_Acc_Inc +  min_dv_Acc_Dec)< dvGoal)
		{
			//Jerk < 0
			while((dv + (a-limits.j_max*ts)*ts) > (dvGoal - CalculateSpeedOnAccelerationChange(a-limits.j_max*ts,0)))
			{
				a -= limits.j_max*ts;
				dv += a*ts;
				aTraj.push_back(a);
			}
			if ((dv + a*ts) > (dvGoal - CalculateSpeedOnAccelerationChange(a,0)))
			{
				dv += a*ts;
				aTraj.push_back(a);
			}
			//Jerk > 0
			while((a+limits.j_max*ts) < -0.0001)
			{
				a += limits.j_max*ts;
				dv += a*ts;
				aTrajTemp.push_back(a);
			}
			double aLast = (dvGoal - dv)/ts;
			if (aLast< -0.0001)
			{
				aTrajTemp.push_back(aLast);
				//aLast can violate the jerk limit, so it is inserted into the right place in the vector
				std::sort (aTrajTemp.begin(), aTrajTemp.end());
			}
			aTraj.insert(std::end(aTraj), std::begin(aTrajTemp), std::end(aTrajTemp));
		}
		//if A_max can be reached
		else
		{
			//Jerk < 0
			while((a-limits.j_max*ts) > -limits.a_max)
			{
				a -= limits.j_max*ts;
				aTraj.push_back(a);
				dv += a*ts;
			}
			//a = min
			a = -limits.a_max;
			while((dv + min_dv_Acc_Inc+a*ts) >= dvGoal)
			{
				dv += a*ts;
				aTraj.push_back(a);
			}
			//Jerk > 0
			while((a+limits.j_max*ts) < -0.0001)
			{
				a += limits.j_max*ts;
				dv += a*ts;
				aTrajTemp.push_back(a);
			}
			double aLast = (dvGoal - dv)/ts;
			if (aLast<-0.0001)
			{
				aTrajTemp.push_back(aLast);
				//aLast can violate the jerk limit, so it is inserted into the right place in the vector
				std::sort(aTrajTemp.begin(), aTrajTemp.end());
			}
			aTraj.insert(std::end(aTraj), std::begin(aTrajTemp), std::end(aTrajTemp));
		}
	}
	return aTraj;
}


