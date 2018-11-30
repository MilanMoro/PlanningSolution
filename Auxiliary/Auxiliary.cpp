#include "Auxiliary.h"
#include "RoutePlanner.h"

#include <stdlib.h>
#include <limits>
#include <math.h>
#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>

template<typename T1, typename T2>
OptimizationResult OptimizeChemotaxis(T1* baseClass, unsigned int numIterations, std::vector<OptimizationArgumentOption> argumentOptions, double (T1::*costFunction)(std::vector<double>,T2), T2 additionalCostArguments)
{
	//This function minimizes the provided cost function (which is a member function of a class T) with a simple stochastic method
	//TODO Create templates for the cost function instead of fix double additional argument vector

	std::vector<double> result;
	srand (time(NULL));

	for (auto argOptionIterator = argumentOptions.begin() ; argOptionIterator != argumentOptions.end(); ++argOptionIterator)
	{
		if ((*argOptionIterator).initialGuessFlag)
		{
			result.push_back((*argOptionIterator).initialGuess);
		}
		else
		{
			//(double(rand())/RAND_MAX) -> [0,1]
			result.push_back((double(rand())/RAND_MAX)*((*argOptionIterator).max-(*argOptionIterator).min)+(*argOptionIterator).min);
		}
	}
	double actualCost = (baseClass->*costFunction)(result,additionalCostArguments);

	for(unsigned int i=0; i<numIterations; i++)
	{
		std::vector<double> tempResult = result;
		for (size_t j=0; j<result.size(); j++)
		{
			//(double(rand())/RAND_MAX)*2-1) -> [-1,1]
			tempResult[j]+= ((double(rand())/RAND_MAX)*2-1)*argumentOptions[j].deviation*(argumentOptions[j].max-argumentOptions[j].min);
			if (tempResult[j] > argumentOptions[j].max)
			{
				tempResult[j] = argumentOptions[j].max;
			}
			else if (tempResult[j] < argumentOptions[j].min)
			{
				tempResult[j] = argumentOptions[j].min;
			}
		}
		double tempCost = (baseClass->*costFunction)(tempResult,additionalCostArguments);
		if(tempCost < actualCost)
		{
			actualCost = tempCost;
			result = tempResult;
		}
	}

	return {result, actualCost};
}

OptimizationResult OptimizeChemotaxis(unsigned int numIterations, std::vector<OptimizationArgumentOption> argumentOptions, double (&costFunction)(std::vector<double>,std::vector<double>),std::vector<double> additionalCostArguments)
{
	//This function minimizes the provided cost function (which is a member function of a class T) with a simple stochastic method
	//TODO Create templates for the cost function instead of fix double additional argument vector

	std::vector<double> result;

	for (auto argOptionIterator = argumentOptions.begin() ; argOptionIterator != argumentOptions.end(); ++argOptionIterator)
	{
		if ((*argOptionIterator).initialGuessFlag)
		{
			result.push_back((*argOptionIterator).initialGuess);
		}
		else
		{
			//(double(rand())/RAND_MAX) -> [0,1]
			result.push_back((double(rand())/RAND_MAX)*((*argOptionIterator).max-(*argOptionIterator).min)+(*argOptionIterator).min);
		}
	}
	double actualCost = costFunction(result,additionalCostArguments);

	for(unsigned int i=0; i<numIterations; i++)
	{
		std::vector<double> tempResult = result;
		for (size_t j=0; j<result.size(); j++)
		{
			//(double(rand())/RAND_MAX)*2-1) -> [-1,1]
			tempResult[j]+= ((double(rand())/RAND_MAX)*2-1)*argumentOptions[j].deviation*(argumentOptions[j].max-argumentOptions[j].min);
			if (tempResult[j] > argumentOptions[j].max)
			{
				tempResult[j] = argumentOptions[j].max;
			}
			else if (tempResult[j] < argumentOptions[j].min)
			{
				tempResult[j] = argumentOptions[j].min;
			}
		}
		double tempCost = costFunction(tempResult,additionalCostArguments);
		if(tempCost < actualCost)
		{
			actualCost = tempCost;
			result = tempResult;
		}
	}

	return {result, actualCost};
}



std::vector<double> DiscreteIntegrator(std::vector<double> timeSeries, double initialValue, double ts)
{
	std::vector<double> integratedTimeSeries = timeSeries;
	std::partial_sum(timeSeries.begin(), timeSeries.end(), integratedTimeSeries.begin());
	for (auto it = integratedTimeSeries.begin() ; it != integratedTimeSeries.end(); ++it)
	{
		*it = *it*ts + initialValue;
	}
	return integratedTimeSeries;
}


template OptimizationResult OptimizeChemotaxis(RoutePlanner* baseClass, unsigned int numIterations, std::vector<OptimizationArgumentOption>, double (RoutePlanner::*costFunction)(std::vector<double>,int),int additionalCostArguments);

