#include <math.h>
#include <stdlib.h>
#include <numeric>
#include "gtest/gtest.h"

#include "Auxiliary.h"

double Rosenbrock(std::vector<double> arg, std::vector<double> argAdd)
{
	return pow((1-arg[0]),2)+100*pow((arg[1]-arg[0]*arg[0]),2);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST (OptimizationTests, Rosenbrock)
{
	std::vector<OptimizationArgumentOption> arg;
	arg.push_back({0,2.4,0.1,0,false});
	arg.push_back({0,2.4,0.1,0,false});
	auto result =  OptimizeChemotaxis(10000, arg, Rosenbrock, {});
    EXPECT_LE(abs(result.optimalArguments[0]-1),0.05);
    EXPECT_LE(abs(result.optimalArguments[1]-1),0.05);
}






