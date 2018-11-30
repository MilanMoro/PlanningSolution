#ifndef AUXILIARY_H_
#define AUXILIARY_H_

#include <vector>

struct OptimizationArgumentOption
{
	double min;
	double max;
	double deviation;
	double initialGuess;
	bool initialGuessFlag;
};

struct OptimizationResult
{
	std::vector<double> optimalArguments;
	double cost;
};

OptimizationResult OptimizeChemotaxis(unsigned int numIterations, std::vector<OptimizationArgumentOption>, double (&costFunction)(std::vector<double>,std::vector<double>),std::vector<double> additionalCostArguments);

template<typename T1, typename T2>
OptimizationResult OptimizeChemotaxis(T1* baseClass, unsigned int numIterations, std::vector<OptimizationArgumentOption>, double (T1::*costFunction)(std::vector<double>,T2),T2 additionalCostArguments);

std::vector<double> DiscreteIntegrator(std::vector<double> timeSeries, double initialValue, double ts);


#endif /* AUXILIARY_H_ */
