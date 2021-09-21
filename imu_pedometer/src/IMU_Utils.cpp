#include "IMU_Utils.h"
#include "DspFilters/Butterworth.h"

using namespace arma;

int IMU_Utils::calculateOdometry(mat &shsignal, mat &shposition, mat&lastpositions, double ax_ref, double ay_ref, double az_ref, bool isbody, pthread_mutex_t &mutex)
{
	Mat_Utils mut;
	// set sampling rate of Phidget
	double deltaT = 8 / 1000;

	// step length
	double STEP_SIZE = 0.60;

	// define IMU constants
	double MAG_MAX = 5.5;
	pthread_mutex_lock(&mutex);
	//cout<<"vecs "<<shsignal.n_rows<<" "<<shsignal.n_cols<<endl;fflush(stdout);
	vec ax = shsignal.col(0);
	vec ay = shsignal.col(1);
	vec az = shsignal.col(2);
	vec mx = shsignal.col(3);
	vec my = shsignal.col(4);
	vec mz = shsignal.col(5);
	vec gx = shsignal.col(6);
	vec gy = shsignal.col(7);
	vec gz = shsignal.col(8);
	vec tstamps = shsignal.col(9);
	pthread_mutex_unlock(&mutex);
	//vec min = shsignal.col(10);
	//vec sec = shsignal.col(11);
	//cout<<"mag "<<endl;fflush(stdout);
	// clear compass data from outlier values
	for (int i = 1; i < mx.n_rows; i++) {
		if (mx(i) > MAG_MAX)
			mx(i) = mx(i - 1);
		if (my(i) > MAG_MAX)
			my(i) = my(i - 1);
		if (mz(i) > MAG_MAX)
			mz(i) = mz(i - 1);
	}

	mat Hd;			

	vec clean_ay, clean_gy;

	// apply the filter on the accelerometer signals
	//cout<<"filter "<<endl;
	clean_ay = filter(Hd, ay);
	
	clean_gy = filter(Hd, gy);

	int steps = 0;
	mat steps_inds;
	//cout << "steps " << endl;
	fflush(stdout);

	// detect the peaks and respective indexes in the filtered signal (accelerometer)
	steps = findPeaks(clean_ay, steps_inds);


	if(steps>1)
	{
		cout<<"STEPS2: "<<steps_inds<<endl;fflush(stdout);
		
		// total number of samples of a single acquisition
		int numSamples = ay.n_elem;

		// this array contains all the compass bearings
		mat all_bearings = zeros<mat>(numSamples, 1);

		// fill bearings array
		for (int i = 0; i < numSamples; i++) {
			// See the inner function for others attitudes
			all_bearings(i) = calculateCompassBearing(ax_ref, ay_ref, az_ref, mx(i), my(i), mz(i));
		}

		if(isbody)
			all_bearings = mut.mod((all_bearings+M_PI/2), 2*M_PI);
		else
			all_bearings = mut.mod(all_bearings, 2*M_PI);
		//----------------  GYROSCOPE PART ------------------------
		cout << "Gyro " << endl;
		fflush(stdout);
		//compute angles from gyroscope data
		mat all_angles = zeros<mat>(numSamples, 1);

		//set same starting angle of the compass
		all_angles(0) = all_bearings(0);

		int first_angle_ind = 0;

		mat clean_gyro = clean_gy; //gyroscope axis where to compute angles

		////ignoring null samples due to gyroscope zeroing
		for (int i = 0; i < numSamples; i++) {
			if ((gy(i) != 0) && (i > 69)) {
				all_angles(i) = all_bearings(i);
				first_angle_ind = i;
				break;
			} else
				all_angles(i) = all_bearings(i);
		}
		//cout << "ignoring null samples " << endl;
		//fflush(stdout);

		//computing angular position from angular velocity (from gyro data)
		for (int i = (first_angle_ind) + 1; i < numSamples; i++) {
			all_angles(i) = all_angles(i - 1) - 0.5 * (clean_gyro(i - 1) + clean_gyro(i)) * deltaT;
		}
		//cout << "angular pos " << endl;
		//fflush(stdout);

		mat all_angles_mean = zeros<mat>(numSamples, 1);
		for (int i = 0; i < numSamples; i++)
			all_angles_mean(i) = mut.angularMean(all_bearings(i), all_angles(i));
		//cout << "media angular " << endl;
		fflush(stdout);

		//mat timeStamps = zeros<mat>(numSamples, 1);

		//for (int i = 0; i < numSamples; i++)
			//timeStamps(i) = //mut.convertTime(hour(i), min(i), sec(i));
		//cout << "timestamp " << endl;
		//fflush(stdout);

		int numSteps = steps;
		mat stepsTimes = mut.subwithmat((vec)tstamps, (vec)steps_inds);//mut.subwithmat(timeStamps, steps_inds); //timeStamps(steps_inds);
		//cout << "steptimes " <<steps<< endl;
		fflush(stdout);

		mat segm_numSamples = zeros<mat>(steps_inds.n_elem, 1);

		segm_numSamples(0) = steps_inds(0);
		for (int i = 1; i < steps_inds.n_elem; i++)
			segm_numSamples(i) = steps_inds(i) - steps_inds(i - 1);
		//cout<<"segm_numSamples "<<segm_numSamples.n_elem<<endl;fflush(stdout);
		//std::cin>>yy;

		int j = 0;

		double one_instant_displacement = STEP_SIZE / segm_numSamples(0);
		//cout << "stepind " << steps_inds<< endl;
		//fflush(stdout);
		for (int i = 1; i < steps_inds(steps_inds.n_elem - 1); i++) {
			if (i > steps_inds(j)) {
				j++;
				if (j >= steps_inds.n_elem)
					break;

				one_instant_displacement = STEP_SIZE / segm_numSamples(j);
				//cout << "one_instant_displacement " << one_instant_displacement<< endl;
				fflush(stdout);
			}
			
			double prevx = 0;
			double prevy = 0;
			if(shposition.n_rows>0)
			{
				prevx = shposition(shposition.n_rows-1,0);
				prevy = shposition(shposition.n_rows-1,1);
			}

			double x = prevx + one_instant_displacement* cos(mut.mod((M_PI*5/2 - all_bearings(i)), M_PI*2));
			double y = prevy + one_instant_displacement* sin(mut.mod((M_PI*5/2 - all_bearings(i)), M_PI*2));
			mat temppos;
			temppos << x << y << one_instant_displacement << mut.mod((M_PI*5/2 - all_bearings(i)),M_PI*2) << tstamps(i);
			shposition.insert_rows(shposition.n_rows, temppos);
			lastpositions.insert_rows(lastpositions.n_rows, temppos);
			
			if(shposition.n_rows>=499)
				shposition.shed_rows(0,shposition.n_rows-249);
		}
		
		cout<<"Shed "<<shsignal.n_rows<<endl;
		pthread_mutex_lock(&mutex);
		if(steps_inds.n_rows>1)
			shsignal.shed_rows(0,steps_inds(steps_inds.n_elem - 1));
		pthread_mutex_unlock(&mutex);
		cout<<"AfterShed "<<shsignal.n_rows<<endl;
	}
	else
	{
		if(shsignal.n_rows>500)
		{
			int numSamples = ay.n_elem;
				// this array contains all the compass bearings
			mat all_bearings = zeros<mat>(numSamples, 1);

			// fill bearings array
			for (int i = 0; i < numSamples; i++) {
				// See the inner function for others attitudes
				all_bearings(i) = calculateCompassBearing(ax(i), ay(i), az(i), mx(i), my(i), mz(i));
			}

			if(isbody)
				all_bearings = mut.mod((all_bearings+M_PI/2), 2*M_PI);
			else
				all_bearings = mut.mod(all_bearings, 2*M_PI);
			//----------------  GYROSCOPE PART ------------------------
			cout << "Gyro " << endl;
			fflush(stdout);
			//compute angles from gyroscope data
			mat all_angles = zeros<mat>(numSamples, 1);

			//set same starting angle of the compass
			all_angles(0) = all_bearings(0);

			int first_angle_ind = 0;

			mat clean_gyro = clean_gy; //gyroscope axis where to compute angles

			////ignoring null samples due to gyroscope zeroing
			for (int i = 0; i < numSamples; i++) {
				if ((gy(i) != 0) && (i > 69)) {
					all_angles(i) = all_bearings(i);
					first_angle_ind = i;
					break;
				} else
					all_angles(i) = all_bearings(i);
			}
			cout << "ignoring null samples " << endl;
			fflush(stdout);

			//computing angular position from angular velocity (from gyro data)
			for (int i = (first_angle_ind) + 1; i < numSamples; i++) {
				all_angles(i) = all_angles(i - 1) - 0.5 * (clean_gyro(i - 1) + clean_gyro(i)) * deltaT;
			}
			cout << "angular pos " << endl;
			fflush(stdout);

			mat all_angles_mean = zeros<mat>(numSamples, 1);
			for (int i = 0; i < numSamples; i++)
				all_angles_mean(i) = mut.angularMean(all_bearings(i), all_angles(i));
				
				
				
			for(int i=0; i<250; i++)
			{
				double prevx = 0;
				double prevy = 0;
				if(shposition.n_rows>0)
				{
					prevx = shposition(shposition.n_rows-1,0);
					prevy = shposition(shposition.n_rows-1,1);
				}

				double one_instant_displacement =0;
				double x = prevx + one_instant_displacement* cos(mut.mod((M_PI*5/2 - all_bearings(i)), M_PI*2));
				double y = prevy + one_instant_displacement* sin(mut.mod((M_PI*5/2 - all_bearings(i)), M_PI*2));
				mat temppos;
				temppos << x << y << one_instant_displacement << mut.mod((M_PI*5/2 - all_bearings(i)),M_PI*2) << tstamps(i);
				shposition.insert_rows(shposition.n_rows, temppos);
				lastpositions.insert_rows(lastpositions.n_rows, temppos);
				
				if(shposition.n_rows>=499)
					shposition.shed_rows(0,shposition.n_rows-249);
			}
			cout<<"Shed "<<shsignal.n_rows<<endl;
			pthread_mutex_lock(&mutex);
			shsignal.shed_rows(0,250);
			pthread_mutex_unlock(&mutex);
			cout<<"AfterShed "<<shsignal.n_rows<<endl;
		}
	}
	
	return steps-1;
}


int IMU_Utils::findPeaks(vec &signal, /*mat difference, mat threshold, mat type,*/ mat& peakInds)	//, mat peakMags)
{
	Mat_Utils mut;

	//cout << "signal " << signal.size() << endl;
	fflush(stdout);
	//cout << "signal " << max(signal) << endl;
	fflush(stdout);
	mat peakMags; //Risultato non usato

	double eps = 2.2204e-16;

	int s = signal.n_elem; //size(signal);
	bool flipData = false; //s(1) < s(2);
	int len0 = signal.n_elem; //numel(signal);
	//if (len0 != s(1) && len0 != s(2))
	//  cout<<"PEAKFINDER:Input "<<"The input data must be a vector"<<endl;

	//if (difference.n_elem==0)
	double difference = (max(signal) - min(signal)) / 5;

	//if (threshold.n_elem==0)
	double threshold = -1.15;

	//if (type.n_elem)
	int type = -1;

	signal = type * signal; // Make it so we are finding maxima regardless
	threshold = threshold * type; // Adjust threshold according to type.
	mat dx0 = diff(signal);//mut.diff(signal); // Find derivative
	//dx0(dx0 == 0) = -eps; // This is so we find the first of repeated values
	mut.maskWithNum(dx0, mut.maskByNum(dx0, 0), -eps);
	//cout << "dx0 " << dx0.n_rows<< endl;
	fflush(stdout);
	int last = dx0.n_rows;
	//vec dx0v = vectorise(dx0);

	//mat ind = mut.find_criteria(dx0v.subvec(0, last - 2), '<', 0) + 1; // Find where the derivative changes sign
	mat searchdev = dx0.rows(0,last-2)%dx0.rows(1,last-1);
	uvec uind =find(searchdev<0);

	mat ind = zeros<mat>(uind.n_elem,1);
	for(int i =0; i<uind.n_elem; i++)
		ind(i)=uind(i);

	//cout << "ind " << ind.size() << endl;
	fflush(stdout);
	// Include endpoints in potential peaks and valleys is desired
	bool include_endpoints = true;
	mat xs;
	if(include_endpoints)
	{
		xs = zeros<mat>(ind.n_elem+2,1);
		xs(0)=signal(0);
		xs(ind.n_elem+1)=len0;
		mat temp = mut.subwithmat((vec) signal, (vec) ind);
		for(int i =0; i<temp.n_elem; i++)
			xs(i+1)= temp(i);

		ind.insert_rows(0,1);
		ind(0)=1;
		ind.insert_rows(ind.n_rows-1,1);
		ind(ind.n_elem-1)=len0;
		//xs << signal(1)<<signal(ind)<<signal(end);
		//ind << 1 << ind <<len0;
	}
	else
		xs = mut.subwithmat((vec) signal,(vec) ind); //signal(ind);
	//cout << "xs " << xs.size() << endl;
	// x only has the peaks, valleys, and possibly endpoints
	int len = xs.n_elem;
	double minMag = (double) min(min(xs));
	double leftMin;

	if (len > 2) // Function with peaks and valleys
	{
		// Set initial parameters for loop
		double tempMag = minMag;
		bool foundPeak = false;
		leftMin = minMag;

		if (include_endpoints) {
			// Deal with first point a little differently since tacked it on
			// Calculate the sign of the derivative since we taked the first point
			//  on it does not neccessarily alternate like the rest.

			//mat signDx = sign(mut.diff(mut.sub_vectorise(xs, 0, 2)));
			mat signDx = sign(diff(xs.rows(0,2)));
			//cout << "signdx" << endl;
			fflush(stdout);
			if (signDx(0) <= 0) // The first point is larger or equal to the second
			{
				if (signDx(0) == signDx(1)) // Want alternating signs
				{
					//mut.del_vectorise(xs, 1);
					//mut.del_vectorise(ind, 1);
					xs.shed_row(1);
					ind.shed_row(1);
					len = len - 1;
				}
				//cout << "deleted" << endl;
				fflush(stdout);
			} else // First point is smaller than the second
			{
				if (signDx(0) == signDx(1)) // Want alternating signs
				{
					//mut.del_vectorise(xs, 0);
					//mut.del_vectorise(ind, 0);
					xs.shed_row(0);
					ind.shed_row(0);
					len = len - 1;
				}
			}
		}

		// Skip the first point if it is smaller so we always start on a
		//   maxima
		int ii = 0;
		if (xs(0) > xs(1))
			ii = -1;
		else
			ii = 0;

		// Preallocate max number of maxima
		int maxPeaks = ceil(len / 2);
		mat peakLoc = zeros<mat>(maxPeaks, 1);
		mat peakMag = zeros<mat>(maxPeaks, 1);
		//cout << "peakLoc " << peakLoc.size() << endl;
		fflush(stdout);

		int cInd = 0;

		//mat tempMag;
		int tempLoc;

		// Loop through extrema which should be peaks and then valleys
		while (ii < len) {
			ii = ii + 1; // This is a peak
			// Reset peak finding if we had a peak and the next peak is bigger
			//   than the last or the left min was small enough to reset.
			if (foundPeak) {
				tempMag = minMag;
				foundPeak = false;
			}

			// Make sure we don't iterate past the length of our vector
			if (ii >= len - 1) {
				break; // We assign the last point differently out of the loop
			}

			// Found new peak that was lager than temp mag and selectivity larger
			//   than the minimum to its left.
			//double xsii = (double) xs(ii);
			
			if ((xs(ii) > tempMag) && (xs(ii) > leftMin + difference)) {

				//minimum distance between two steps (60 samples)
				if (cInd > 0) {
					if (ind(ii) - ind(peakLoc(cInd - 1)) > 60) {
						tempLoc = ii;
						tempMag = xs(ii);
					}
				} else {
					tempLoc = ii;
					tempMag = xs(ii);
				}
			}
			ii = ii + 1; // Move onto the valley
			if(ii>xs.n_elem-1)
			{
				//cout<<"MAGGIORE "<<len<<endl;
				break;
			}
			// Come down at least difference from peak
			if (!foundPeak && (tempMag > difference + xs(ii))) {
				foundPeak = true; // We have found a peak
				leftMin = xs(ii);
				peakLoc(cInd) = tempLoc; // Add peak to index
				peakMag(cInd) = tempMag;
				cInd = cInd + 1;
			} else {
				if (xs(ii) < leftMin) // New left minima
					leftMin = xs(ii);

			}

		}
		

		// Check end poin
		if (xs(xs.n_elem - 1) > tempMag
				&& xs(xs.n_elem - 1) > leftMin + difference)
		{
			peakLoc(cInd) = len-1;
			peakMag(cInd) = xs(xs.n_elem - 1);
			cInd = cInd + 1;
		} else {
			if (!foundPeak && tempMag > minMag) // Check if we still need to add the last point
			{
				peakLoc(cInd) = tempLoc;
				peakMag(cInd) = tempMag;
				cInd = cInd + 1;
			}
		}

		// Create output
		if(cInd>1)
		{
			
			//cout << "cInd>0"<<cInd<<endl;
			peakInds = mut.subwithmat(ind,peakLoc.rows(0,cInd-2));
			peakMags = peakMag.rows(0,cInd-2);
			//peakInds = mut.subwithmat(ind, mut.sub_vectorise(peakLoc, 0, cInd - 1));
			//peakMags = mut.sub_vectorise(peakMag, 0, cInd - 1);
		}


		//cout<<"cInd"<<cInd<<endl;
	}
	else // This is a monotone function where an endpoint is the only peak
	{
		long long unsigned int xInd;
		double dpeakMags= xs.max(xInd);

		if (dpeakMags > minMag + difference)
			peakInds = ind(xInd);
		else
		{
			peakMags.reset();
			peakInds.reset();
		}
	 }

	// Apply threshold value.  Since always finding maxima it will always be
	//   larger than the threshold.
	/*if (threshold!=0)//(!threshold.is_empty())
	 {
	 bool m = peakMags>threshold;
	 peakInds = peakInds(m);
	 peakMags = peakMags(m);
	 }*/

	// Rotate data if needed
	if (flipData) {
		peakMags = peakMags.t();
		peakInds = peakInds.t();
	}

	// Change sign of data if was finding minima
	if (type < 0) {
		for(int i=0; i<peakMags.n_elem; i++)
			peakMags(i) = -peakMags(i);
		for(int i =0; i<signal.n_elem; i++)
			signal(i) = -signal(i);
	}

	// // Plot
	// if isempty(peakInds)
	//     disp('No significant peaks found')
	// else
	//     figure;
	//     plot(1:len0,signal,'.-',peakInds,peakMags,'ro','linewidth',2);
	// end

	int nPeaks = peakInds.size();
	//cout<<"nPeaks"<<nPeaks<<endl;
	return nPeaks;	// nPeaks;
}

//// Funzione che restituisce una direzione geografica con i dati IMU
double IMU_Utils::calculateCompassBearing(double a0, double a1, double a2,
		double m0, double m1, double m2) {

	// a0=0 a1=0 a2=1   --> Horizontal position
	// a0=0 a1=1 a2=0   --> Vertical position with usb upward
	// a0=0 a1=-1 a2=0  --> Vertical position with usb downward
	// a0=1 a1=0 a2=0   --> Left side down
	// a0=-1 a1=0 a2=0  --> Right side down

	//double rollAngle = atan2(a1, a2);
	//double pitchAngle = atan(-a0 / (a1 * sin(rollAngle) + a2 * cos(rollAngle)));
	//double yawAngle = atan2((m2 * sin(rollAngle) - m1 * cos(rollAngle)),(m0 * cos(pitchAngle) + m1 * sin(pitchAngle) * sin(rollAngle) + m2 * sin(pitchAngle) * cos(rollAngle)));
	double yawAngle = atan2(m2,m0);

	double bearing = yawAngle;
	

	if (bearing < 0)
		bearing = bearing + 2.0 * M_PI;
	cout<<bearing<<endl;
	return bearing;

	//code to translate in degrees between 0 - 360�

	//bearing = (yawAngle * (180.0 / pi));
	//if (bearing < 0)
	//    bearing = bearing + 360;
	//end
	//display(bearing)
}

vec IMU_Utils::filter(mat& type, vec &toFilt) {
	//return toFilt;

	mat A = toFilt.t();
	Mat_Utils mut;
	float **arr = mut.matToFloat(A);

	Dsp::SimpleFilter<Dsp::Butterworth::LowPass<6>,1> f;

	int ord = 6, samR = 40, cut = 5;

	f.setup(ord, samR, cut);

	f.process(A.n_cols, arr);

	mat filtA = mut.floatToMat(arr, A.n_rows, A.n_cols);

	vec retvec = filtA.t();
	//cout<<toFilt.n_rows<<"x"<<toFilt.n_cols<<endl;
	//cout<<retvec.n_rows<<"x"<<retvec.n_cols<<endl;
	//std::cin>>ord;

	return retvec;

}

/*
int mains() {
	IMU_Utils iu;
	mat passi;
	iu.getIMUData("Motion_vicoli_walk_1.txt", passi, true);
	iu.getIMUData("Head_vicoli_walk_1.txt", passi, false);
	
	cout<<"passi "<<passi.n_rows;
}*/

