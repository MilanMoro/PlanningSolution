#ifndef OBSTACLEPREDICTOR_H_
#define OBSTACLEPREDICTOR_H_

#include <vector>
#include <map>

struct Vehicle
{
	double naiveEstArrive;
	double naiveEstLeave;
	double estArrive;
	double estLeave;
};

class ObstaclePredictor {
public:
	ObstaclePredictor(double uncertinity, double crossStart, double crossEnd, double timeStep)
			:uncertinityConstant(uncertinity), crossingStart(crossStart), crossingEnd(crossEnd), ts(timeStep){};

	void AddVehicleLeft(double pos, double v);
	void AddVehicleOpposite(double pos, double v);
	void AddVehicleRight(double pos, double v);
	std::vector<std::pair<double,double>> ProvideConstraints();
	void DeleteCars();

private:

	//uncertinityConstant should be not be negative, 0 meaning absolute certainty
	double uncertinityConstant; //-> t1*exp(-t*c), t2*exp(t*c)
	double crossingStart;
	double crossingEnd;
	double ts;

	double followerTimeAddition = 2;
	double rightHandTimeAddition = 3;

	std::map<double,Vehicle,std::greater<double>> LeftVehicles;
	std::map<double,Vehicle,std::greater<double>> OppositeVehicles;
	std::map<double,Vehicle,std::greater<double>> RightVehicles;

	void CalculateEstimations();

};

#endif /* OBSTACLEPREDICTOR_H_ */
