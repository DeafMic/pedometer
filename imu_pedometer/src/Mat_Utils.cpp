#include "Mat_Utils.h"

using namespace arma;

mat Mat_Utils::diff(mat X) {
	mat Y = zeros<mat>(X.n_rows - 1, X.n_cols);
	for (int i = 0; i < X.n_cols; i++) {
		for (int j = 0; j < X.n_rows - 1; j++) {
			Y(j, i) = X(j + 1, i) - X(j, i);
		}
	}
	return Y;
}

vec Mat_Utils::find(mat X) {
	vec Y = zeros<vec>(1);
	int count = 0;
	int ind = 0;
	for (int i = 0; i < X.n_rows; i++) {
		for (int j = 0; j < X.n_cols; j++) {
			ind++;
			if (X(i, j) != 0) {
				Y.resize(count + 1, 1);
				Y(count) = ind;
				count++;
			}
		}
	}
	return Y;
}

vec Mat_Utils::find_criteria(mat X, char cr, double val) {
	vec Y = zeros<vec>(1);
	int count = 0;
	int ind = 0;
	for (int i = 0; i < X.n_rows; i++) {
		for (int j = 0; j < X.n_cols; j++) {
			ind++;
			switch (cr) {
			case '<':
				if (X(i, j) < val) {
					Y.resize(count + 1, 1);
					Y(count) = ind;
					count++;
				}
				break;

			case '>':
				if (X(i, j) > val) {
					Y.resize(count + 1, 1);
					Y(count) = ind;
					count++;
				}
				break;
			case '=':
				if (X(i, j) == val) {
					Y.resize(count + 1, 1);
					Y(count) = ind;
					count++;
				}
				break;
			default:
				if (X(i, j) != 0) {
					Y.resize(count + 1, 1);
					Y(count) = ind;
					count++;
				}
				break;
			}

		}
	}
	return Y;
}


mat Mat_Utils::sub_vectorise(mat X, int start, int stop)
{
	vec Y = vectorise(X);
	mat Z = Y.subvec(start, stop);
	return Z;
}

mat Mat_Utils::del_vectorise(mat X, int elem)
{
	vec Y = vectorise(X);
	int vpos=0;
	mat Z = zeros<mat>(Y.n_rows-1,1);
	for(int i=0;i<Y.n_rows;i++)
	{
		if(i!=elem)
		{
			Z(vpos,0)=Y(i);
			vpos++;
		}
	}
	return Z;
}

mat Mat_Utils::smooth_moving(mat x, mat y, int span)
{

	return x;
}

mat Mat_Utils::xNumber(mat X, double p)
{
	mat Y = zeros<mat>(X.n_rows, X.n_cols);
		for(int i=0; i<X.n_rows; i++)
			for(int j=0; j<X.n_cols;j++)
				Y(i,j)=X(i,j)*p;
		return Y;
}

mat Mat_Utils::mod(mat x, double div)
{
	mat Y = zeros<mat>(x.n_rows, x.n_cols);
	for(int i=0; i<x.n_rows; i++)
		for(int j=0; j<x.n_cols;j++)
			Y(i,j)=mod(x(i,j),div);
	return Y;
}

mat Mat_Utils::mod(mat x, double add, double div)
{
	mat Y = zeros<mat>(x.n_rows, x.n_cols);
	for(int i=0; i<x.n_rows; i++)
		for(int j=0; j<x.n_cols;j++)
			Y(i,j)=mod((x(i,j)+add),div);
	return Y;
}

double Mat_Utils::mod (double x, double y)
{
	double n = floor(x/y);
	return x - n*y;

}

double Mat_Utils::angularMean (double a1, double a2)
{
	double mean = (a1 + a2) / 2;
	if (abs(a1 - a2) > 180)
		mean = mean - 180;

	return mod(mean,2*M_PI);
}

int Mat_Utils::convertTime(int hours,int mins, double secs)
 {
	return hours*60*60*1000 + mins*60*1000 + secs*1000;
 }

mat Mat_Utils::set_subvector(mat X, mat Y, int start, int stop)
{
	mat Z=zeros<mat>(X.n_cols);
	for(int i = 0; i<Y.n_elem && start+i<stop; i++)
		Z(start+i)=Y(i);
	return Z;
}

mat Mat_Utils::subwithmat(mat X, mat Y)
{
	mat Z=zeros<mat>(Y.n_elem,1);
	for(int i = 0; i<Y.n_elem; i++)
		Z(i)=X(Y(i));

	return Z;
}

vec Mat_Utils::subwithmat(vec X, vec Y)
{
	vec Z=zeros<vec>(Y.n_elem,1);
	for(int i = 0; i<Y.n_elem; i++)
		Z(i)=X(Y(i));

	return Z;
}

cube Mat_Utils::subcubeByIndex(cube A, uvec idx)
{
	cube res= zeros<cube>(A.n_rows,A.n_cols, idx.n_elem);
	unsigned int sl = 0;
	for(int i=0;i<idx.n_elem; i++)
		res.slice(i)=A.slice(idx(i));
	return res;
}

mat Mat_Utils::mask(mat X, mat Y)
{
	mat Z = zeros<mat>(X.n_rows, X.n_cols);
	for(int i=0; i<X.n_rows; i++)
		for(int j=0; j<X.n_cols;j++)
			if(Y(i,j)==1)
				Z(i,j)=X(i,j);
	return Z;
}

mat Mat_Utils::maskWithNum(mat X, mat Y, double val)
{
	mat Z = zeros<mat>(X.n_rows, X.n_cols);
	for(int i=0; i<X.n_rows; i++)
		for(int j=0; j<X.n_cols;j++)
			if(Y(i,j)==1)
				Z(i,j)=val;
			else
				Z(i,j)=X(i,j);
	return Z;
}

mat Mat_Utils::maskByNum(mat X, double val)
{
	mat Y = zeros<mat>(X.n_rows, X.n_cols);
	for(int i=0; i<X.n_rows; i++)
		for(int j=0; j<X.n_cols;j++)
			if(X(i,j)==val)
				Y(i,j)=1;
	return Y;
}

mat Mat_Utils::sum(mat X)
{
	mat Y = zeros<mat>(1, X.n_cols);
	for(int i=0; i<X.n_cols; i++)
	{
		double sum = 0;
		for(int j=0; j<X.n_rows;j++)
			sum+=X(j,i);
		Y(0,i)=sum;
	}
	return Y;
}

float** Mat_Utils::matToFloat(mat &A) {
	int row = A.n_rows;
	int col = A.n_cols;

	float **fmat = new float*[row];
	for (int i = 0; i < row; i++) {
		fmat[i] = new float[col];
		for (int j = 0; j < col; j++)
			fmat[i][j] = A(i, j);
	}

	return fmat;
}

/**
 * Converte una matrice float** in una matrice Armadillo
 * @param fmat matrice nel formato float**
 * @param row numero di righe
 * @param col numero di colonne
 * @return Matrice nel formato Armadillo
 */
mat Mat_Utils::floatToMat(float** fmat, int row, int col) {
	mat A = zeros<mat>(row, col);

	for (int i = 0; i < row; i++)
		for (int j = 0; j < col; j++)
			A(i, j) = fmat[i][j];

	return A;
}

uvec Mat_Utils::randsample(int range, int numsample, bool usew, vec weights)
{

	//cout<<weights<<endl;
	int dim = weights.n_elem;
	uvec res = zeros<uvec>(numsample);
	for(int i=0; i<numsample; i++)
	{
		double ran = 0.01*(rand()%100);
		//cout<<ran<<endl;
		double wsum=weights(0);
		int j=1;
		while(ran>wsum && j<dim)
		{
			//cout<<" "<<wsum<<endl;
			wsum+=weights(j);
			j++;
		}
		res(i)=j-1;

	}


	/*
	std::vector<double> probabilities(dim);
	for(int i=0; i<dim; i++)
		probabilities.at(i)=weights(i);

	boost::mt19937 gen;
	boost::random::discrete_distribution<> dist(probabilities.begin(), probabilities.end());

	for (int i=0; i<numsample; ++i)
		res(i) = dist(gen);
	 */
	//cout<<res<<endl;
	//getchar();

	return res;
}

int maind() {
	Mat_Utils mut;
	mat A = zeros<mat>(3, 3);
	A << 1 << 2 << 3 << endr << 4 << 5 << 6 << endr << 7 << 8 << 9 << endr;

	//cout<<A;
	//cout << mut.del_vectorise(A, 2);*/


	uvec B; B  << 1 << 1<<1;
	cube C =randi<cube>(3,3,3);
	cout<<A.cols(B)-2;
	//cout<<C<<endl;
	//cout<<mut.subcubeByIndex(C,B);

	//cout<<A.elem(C);
	vec w; w <<0.1<<0.2<<0.1<<0.1<<0.1<<0.1;
	//cout<<mut.randsample(4,10,true,w);
	//cout<<B.max(C)<<endl<<C;

	return 1;
}
