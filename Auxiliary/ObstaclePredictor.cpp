#include <math.h>

#include "ObstaclePredictor.h"


void ObstaclePredictor::AddVehicleLeft(double pos, double v)
{
	Vehicle car;
	car.naiveEstLeave = 0;
	if (pos < crossingEnd)
	{
		car.naiveEstLeave = ceil((crossingEnd-pos)/(v+0.1)/ts)*ts;
	}
	if (pos < crossingStart)
	{
		car.naiveEstArrive = floor((crossingStart-pos)/(v+0.1)/ts)*ts;
	}
	LeftVehicles.insert(std::pair<double,Vehicle>(pos,car));
}

void ObstaclePredictor::AddVehicleOpposite(double pos, double v)
{
	Vehicle car;
	car.naiveEstLeave = 0;
	if (pos < crossingEnd)
	{
		car.naiveEstLeave = ceil((crossingEnd-pos)/(v+0.1)/ts)*ts;
	}
	if (pos < crossingStart)
	{
		car.naiveEstArrive = floor((crossingStart-pos)/(v+0.1)/ts)*ts;
	}
	OppositeVehicles.insert(std::pair<double,Vehicle>(pos,car));
}

void ObstaclePredictor::AddVehicleRight(double pos, double v)
{
	Vehicle car;
	car.naiveEstArrive = 0;
	car.naiveEstLeave = 0;
	// For the right side vehicles the real crossing area starts earlier by around 4 meters
	if (pos < crossingEnd-4)
	{
		car.naiveEstLeave = ceil((crossingEnd-4-pos)/(v+0.1)/ts)*ts;
	}
	if (pos < crossingStart-4)
	{
		car.naiveEstArrive = floor((crossingStart-4-pos)/(v+0.1)/ts)*ts;
	}
	RightVehicles.insert(std::pair<double,Vehicle>(pos,car));
}

void ObstaclePredictor::CalculateEstimations()
{
   for (auto it = RightVehicles.begin(); it != RightVehicles.end(); ++it)
   {
	   double actEstArrive = it->second.naiveEstArrive;
	   double actEstLeave = it->second.naiveEstLeave;
	   it->second.estLeave = actEstLeave*(exp(actEstLeave*uncertinityConstant));
	   it->second.estArrive = actEstArrive*(exp(-actEstArrive*uncertinityConstant));
	   if (it != RightVehicles.begin())
	   {
		   if (it->second.estArrive < std::prev(it,1)->second.estLeave)
		   {
			   it->second.estArrive = std::prev(it,1)->second.estLeave;
		   }
		   if (it->second.estLeave < std::prev(it,1)->second.estLeave)
		   {
			   it->second.estLeave = std::prev(it,1)->second.estLeave + followerTimeAddition;
		   }
	   }

	   // Dummy solution for taking into account other cars:
	   if(!OppositeVehicles.empty())
	   {
		   it->second.estLeave += rightHandTimeAddition;
	   }
   }
}

std::vector<std::pair<double, double> > ObstaclePredictor::ProvideConstraints()
{
	std::vector<std::pair<double, double>> constraints;
	CalculateEstimations();
	for (auto it = RightVehicles.begin(); it != RightVehicles.end(); ++it)
	{
		constraints.push_back({it->second.estArrive, it->second.estLeave});
	}
	DeleteCars();
	return constraints;
}

void ObstaclePredictor::DeleteCars()
{
	LeftVehicles.clear();
	OppositeVehicles.clear();
	RightVehicles.clear();
}
