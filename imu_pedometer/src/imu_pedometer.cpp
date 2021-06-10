#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "IMU_Utils.h"
#include <phidget21.h>
#include <imu_pedometer/imu.h>
#include <geometry_msgs/Twist.h>

std::string turtle_name;


//Porcata
mat imudata;
int serialNo;
ros::Publisher pub1;
ros::Publisher pub2;
int synchro=0;

//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);
//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);

//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid);

int spatial_simple();

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_broadcaster");
	//if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
	turtle_name = "odom";
	
	IMU_Utils imut;
	Mat_Utils mut;
	bool primogiro = true;

	ros::NodeHandle node;
	static tf::TransformBroadcaster br;

	double frequency = 125;
	unsigned int num_readings = 721;
	double laser_frequency = 40;
	mat ranges = zeros<mat>(0,721);
	double intensities[1];

	int count = 0;

	double xb=0, yb=0, thb=0, xh=0, yh=0, thh=0, prevthb=0, thb_offset=0, thh_offset=0, diff_offset=0, thlaser=0;
	int ts1, ts2, ts3;

	ros::Rate loop_rate(frequency);
	
	double dax, day, daz, dmx, dmy, dmz, dgx, dgy, dgz;
	int dhour;
	int dmin;
	double dsec;
	
	//mat short_signal = zeros<mat>(0,10)	;
	//vec tempsignal = zeros<vec>(1);
	//int tempcount=0;
	int totstep=0;
	int totot = 0;
	int totind = 0;
	vec onlinestep;
	
	mat shposition = zeros<mat>(0,5);
	mat lastpositions= zeros<mat>(0,5);
	int totodom=0;
	int odsteps=0;
	
	double ax_ref, ay_ref, az_ref;
	
	tf::Transform transform;
	tf::Transform t2;

	
	/****PHIDGET****/
	int result;
	const char *err;
	
	imudata = zeros<mat>(0,10);

	//Declare a spatial handle
	CPhidgetSpatialHandle spatial = 0;

	//create the spatial object
	CPhidgetSpatial_create(&spatial);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
	



	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, -1);
	//get the program to wait for a spatial device to be attached
	printf("Waiting for spatial to be attached.... \n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}
	
	/*for(int i=0; i<10; i++)
	{
		sleep(1);
		cout<<".";fflush(stdout);
	}*/

	//Display the properties of the attached spatial device
	display_properties((CPhidgetHandle)spatial);

	//read spatial event data
	printf("Reading.....\n");
	
	//Set the data rate for the spatial events
	CPhidgetSpatial_setDataRate(spatial, 8);

	//run until user input is read
	//printf("Press any key to end\n");
	//getchar();
	
	/*****************/
	
	if(serialNo==296835) {
	pub1 = node.advertise<geometry_msgs::Twist>("/body_pose", 1000);
        pub2 = node.advertise<imu_pedometer::imu>("/imubody", 1000);
	synchro=1;
	}

	if(serialNo==296860) {
	pub1 = node.advertise<geometry_msgs::Twist>("/head_pose", 1000);
        pub2 = node.advertise<imu_pedometer::imu>("/imuhead", 1000);
	synchro=1;
	}
	
	
	while(ros::ok())
	{
		
		if(totot==0 && imudata.n_rows==124)
		{
			ax_ref=dax;
			ay_ref=day;
			az_ref=daz;
			totot =1;
		}
		
		
		if(imudata.n_rows>250)
		{		
			
			//cout<<"signal "<<imudata.n_rows<<endl;
			odsteps = imut.calculateOdometry(imudata, shposition, lastpositions, ax_ref, ay_ref, az_ref, false, mutex);
			//cout<<"odstep "<<odsteps<<endl;
		}

		//cout<<"lastpos "<<lastpositions.n_rows<<endl;
		//if(shposition.n_rows>0)
			//cout<<"shposition "<<shposition.n_rows<<" "<<shposition(shposition.n_rows-1,0)<<endl;

		if(lastpositions.n_rows>0)
		{
			//cout<<"invio "<<lastpositions.n_rows<<endl; fflush(stdout);
			/*transform.setOrigin( tf::Vector3(lastpositions(0,0), lastpositions(0,1), 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, lastpositions(0,3));// thh+thp-prevthp);//msg->theta);
			transform.setRotation(q);
			tf::Quaternion q2;
			q2.setRPY(0, 0, 0);
			
			t2.setOrigin( tf::Vector3(0, 0, 0.0) );
			t2.setRotation(q2);
			//ros::Time timestamp(lastpositions(i,4)/1000);
			
			br.sendTransform(tf::StampedTransform(t2, ros::Time::now(), "laser", "base_link"));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odometry_offset", "laser"));*/
			//cout<<"*"; fflush(stdout);

			geometry_msgs::Twist mypose;
			mypose.linear.x=lastpositions(0,0);
			mypose.linear.y=lastpositions(0,1);
			mypose.angular.z=lastpositions(0,3);
			pub1.publish(mypose);
			
			lastpositions.shed_row(0);
		}
		
		loop_rate.sleep();
		ros::spinOnce();
		
	}
	
	
	
	printf("Closing...\n");

	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);
	
	
	ros::shutdown();
	
	
	
	return 0;
};


//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);

	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);

	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	int i;
	/*printf("Number of Data Packets in this event: %d\n", count);
	for(i = 0; i < count; i++)
	{
		printf("=== Data Set: %d ===\n", i);
		printf("Acceleration> x: %6f  y: %6f  z: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
		printf("Angular Rate> x: %6f  y: %6f  z: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
		printf("Magnetic Field> x: %6f  y: %6f  z: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
		printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
	}

	printf("---------------------------------------------\n");*/
	
	for(int i = 0; i < count; i++)
	{
		mat tempmat;
		double time = (double)data[i]->timestamp.seconds+(double)data[i]->timestamp.microseconds/1000000;
		
		tempmat<<data[i]->acceleration[0]<<data[i]->acceleration[1]<<data[i]->acceleration[2]<<
				data[i]->magneticField[0]<<data[i]->magneticField[1]<<data[i]->magneticField[2]<<
				data[i]->angularRate[0]<<data[i]->angularRate[1]<<data[i]->angularRate[2]<<time;
		//cout<<"imudata "<<imudata.n_rows<<endl;
		pthread_mutex_lock(&mutex);
		imudata.insert_rows(imudata.n_rows, tempmat);
		pthread_mutex_unlock(&mutex);
		if (ros::ok() and synchro==1){
		imu_pedometer::imu myimu;
		myimu.tstamp = time;
		myimu.ax=data[i]->acceleration[0];
		myimu.ay=data[i]->acceleration[1];
		myimu.az=data[i]->acceleration[2];
		myimu.mx=data[i]->magneticField[0];
		myimu.my=data[i]->magneticField[1];
		myimu.mz=data[i]->magneticField[2];
		myimu.gx=data[i]->angularRate[0];
		myimu.gy=data[i]->angularRate[1];
		myimu.gz=data[i]->angularRate[2];
		pub2.publish(myimu);}
		//std::cout << "serial NUMBER: " << serialNo << std::endl << std::endl;
	}
	return 0;
}

//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

	

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);

	return 0;
}

int spatial_simple()
{
	int result;
	const char *err;
	
	imudata = zeros<mat>(0,10);

	//Declare a spatial handle
	CPhidgetSpatialHandle spatial = 0;

	//create the spatial object
	CPhidgetSpatial_create(&spatial);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
	



	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, -1);
	/*for(int i=0; i<5; i++)
	{
		sleep(1);
		cout<<".";fflush(stdout);
	}*/
	//get the program to wait for a spatial device to be attached
	/*printf("Waiting for spatial to be attached.... \n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}*/

	//Display the properties of the attached spatial device
	display_properties((CPhidgetHandle)spatial);

	//read spatial event data
	printf("Reading.....\n");
	
	//Set the data rate for the spatial events
	CPhidgetSpatial_setDataRate(spatial, 16);

	//run until user input is read
	printf("Press any key to end\n");
	getchar();
	//while(1)
		//sleep(1);

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);

	return 0;
}


