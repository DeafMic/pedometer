#include <iostream>
#include "Butterworth.h"


int maini()
{
	Dsp::SimpleFilter<Dsp::Butterworth::LowPass<6>,1> f;

	return 1;
}
