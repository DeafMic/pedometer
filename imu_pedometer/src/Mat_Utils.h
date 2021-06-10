#ifndef MAT_UTILS_H_
#define MAT_UTILS_H_

#include <armadillo>
#include <iostream>
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

using namespace arma;

class Mat_Utils
{
public:

	static mat diff(mat X);
	static vec find(mat X);
	static vec find_criteria(mat X, char c, double val);
	static mat sub_vectorise(mat X, int start, int stop);
	static mat set_subvector(mat X, mat Y, int start, int stop);
	static mat del_vectorise(mat X, int elem);
	static mat subwithmat(mat X, mat Y);
	static vec subwithmat(vec X, vec Y);
	static cube subcubeByIndex(cube A, uvec idx);
	//static double radtodeg(double x) { return x/M_PI*180; }
	//static double degtorad(double x) { return x/180*M_PI; }
	static mat smooth_moving(mat x, mat y, int span);
	static mat mod(mat x, double div);
	static mat mod(mat x, double add, double div);
	static double mod (double x, double y);
	static mat xNumber(mat X, double p);
	static mat mask(mat X, mat Y);
	static mat maskWithNum(mat X, mat Y, double val);
	static mat maskByNum(mat X, double val);
	static double angularMean (double a1, double a2);
	static int convertTime(int hours,int mins, double secs);
	static mat sum(mat X);
	static mat floatToMat(float** fmat, int row, int col);
	static float** matToFloat(mat &A);
	static uvec randsample(int range, int numsample, bool usew, vec weights);

};



#endif /* MAT_UTILS_H_ */
