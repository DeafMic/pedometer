#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "IMU_Utils.h"
#include "Spatial-simple.h"

std::string turtle_name;


int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_broadcaster");
	//if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
	turtle_name = "odom";
	
	IMU_Utils imut;
	Mat_Utils mut;
	bool primogiro = true;

	ros::NodeHandle node;
	static tf::TransformBroadcaster br;
	ros::Publisher scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 50);

	double frequency = 125;
	unsigned int num_readings = 721;
	double laser_frequency = 40;
	mat ranges = zeros<mat>(0,721);
	double intensities[1];

	int count = 0;

	double xb=0, yb=0, thb=0, xh=0, yh=0, thh=0, prevthb=0, thb_offset=0, thh_offset=0, diff_offset=0, thlaser=0;
	int ts1, ts2, ts3;
	
	//FILE *pf, *plaser;
	

	//pf = fopen ("/home/user/Desktop/Head_vicoli_walk_1.txt","r");
	//plaser = fopen ("/home/user/Desktop/Laser_vicoli_walk_1.txt","r");

	ros::Rate loop_rate(frequency);
	
	double dax, day, daz, dmx, dmy, dmz, dgx, dgy, dgz;
	int dhour;
	int dmin;
	double dsec;
	
	mat short_signal;
	//vec tempsignal = zeros<vec>(1);
	//int tempcount=0;
	int totstep=0;
	int totot = 0;
	int totind = 0;
	vec onlinestep;
	
	mat shposition = zeros<mat>(0,5);
	int totodom=0;
	int odsteps=0;
	
	double ax_ref, ay_ref, az_ref;
	
	tf::Transform transform;
	tf::Transform t2;
	
	mat imudata = zeros<mat>(0,9);

	while(ros::ok())
	{
		
		//////////////LOOP ACQUISIZIONE////////////////////
		/*if(primogiro)
		{
			int timelaser=0, tmp; 
			fscanf(plaser,"%d;",&timelaser);
			//cout<<"get time "<<tempscan(0,0)<<endl;
			for(int i=0;i<1081; i++)
				fscanf(plaser,"%d;",&tmp);
			fscanf(plaser,"\n");
			fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d:%d:%lf\n", &dax,&day, &daz, &dmx, &dmy, &dmz, &dgx, &dgy, &dgz, &dhour, &dmin,&dsec);
			fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d:%d:%lf\n", &dax,&day, &daz, &dmx, &dmy, &dmz, &dgx, &dgy, &dgz, &dhour, &dmin,&dsec);
			fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d:%d:%lf\n", &dax,&day, &daz, &dmx, &dmy, &dmz, &dgx, &dgy, &dgz, &dhour, &dmin,&dsec);
			
			Mat_Utils mut;
			int t=0;
			while(t<=timelaser)
			{
				if(EOF==fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d:%d:%lf\n", &dax,&day, &daz, &dmx, &dmy, &dmz, &dgx, &dgy, &dgz, &dhour, &dmin,&dsec))
					break;
				t=mut.convertTime(dhour,dmin,dsec);
				printf("primo giro time %d %d\n",t,timelaser);
			}
			
			while(timelaser<t)
			{
				fscanf(plaser,"%d;",&timelaser);
				for(int i=0;i<1081; i++)
					fscanf(plaser,"%d;",&tmp);
				fscanf(plaser,"\n");
				printf("primo giro time %d %d\n",t,timelaser);
			}
			
			primogiro=false;
		}
		
		if(EOF==fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d:%d:%lf\n", &dax,&day, &daz, &dmx, &dmy, &dmz, &dgx, &dgy, &dgz, &dhour, &dmin,&dsec))
				break;
			
		*/
		
		
					
		mat tMat;
		double t = mut.convertTime(dhour,dmin,dsec);
		tMat << dax << day << daz << dmx << dmy << dmz << dgx << dgy << dgz << t;

		short_signal.insert_rows(short_signal.n_rows, tMat);
		
		if(totot==0 && short_signal.n_rows==124)
		{
			ax_ref=dax;
			ay_ref=day;
			az_ref=daz;
		}
		
		//Provo il calcolo dei passi online
		//tempsignal.resize(tempcount+1);
		//tempsignal(tempcount)=day;
		//tempcount++;
		
		
		if(short_signal.n_rows>250)
		{
			// apply a low-pass 6th order Butterworth filter all frequency values are in Hz.			
			mat lastpositions = zeros<mat>(0,5);
			cout<<"signal "<<short_signal.n_rows<<endl;
			odsteps = imut.calculateOdometry(short_signal, shposition, lastpositions, ax_ref, ay_ref, az_ref, false);
			if(lastpositions.n_rows>0)
			{
				for(int i=0; i<odsteps; i++)
					cout<<"+";
				totodom+=odsteps;
				//totot+=short_signal.n_rows;
				//cout<<tempcount<<endl;
				//tempsignal.clear();	
				//if(odsteps>0)
					//short_signal.clear();	
							
				//tempcount=0;
				
				cout<<"Righe "<<ranges.n_rows<<" "<<lastpositions.n_rows<<endl;
				//INVIO I MESSAGGI
				//while(ranges.n_rows>0 && lastpositions.n_rows>0)
				{
					//while(lastpositions(0,4)<ranges(0,0))
					{
						transform.setOrigin( tf::Vector3(lastpositions(0,0), lastpositions(0,1), 0.0) );
						tf::Quaternion q;
						q.setRPY(0, 0, lastpositions(0,3));// thh+thp-prevthp);//msg->theta);
						transform.setRotation(q);
						tf::Quaternion q2;
						q2.setRPY(0, 0, 0);
						
						t2.setOrigin( tf::Vector3(0, 0, 0.0) );
						t2.setRotation(q2);
						//ros::Time timestamp(lastpositions(i,4)/1000);
						
						br.sendTransform(tf::StampedTransform(t2, ros::Time::now(), "laser", "base_link"));
						br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odometry_offset", "laser"));
						cout<<"*"; fflush(stdout);
						
						lastpositions.shed_row(0);
						cout<<"-"; fflush(stdout);
						if(lastpositions.n_rows==0)
							break;
						
					}
					/*//if(lastpositions.n_rows>0)
					{
						//while( lastpositions(0,4)>ranges(0,0))
						{
							cout<<"Laser Sent mat size="<<ranges.n_rows<<endl;fflush(stdout);
							sensor_msgs::LaserScan scan;
							ros::Time scan_time = ros::Time::now();
							scan.header.stamp = scan_time;
							scan.header.frame_id = "base_laser_link";
							scan.angle_min = -1.57079637051;
							scan.angle_max = 1.56643295288;
							scan.angle_increment = 0.00436332309619;
							scan.time_increment = 1.73611115315e-05;
							scan.range_min = 1.73611115315e-05;
							scan.range_max = 60.0;

							scan.ranges.resize(720);
							scan.intensities.resize(0);
							for(unsigned int i = 1; i < 721; ++i){
								float ss = (float)ranges(0,i)/1000;
								scan.ranges[i-1] = ss;
							}

							scan_pub.publish(scan);
								
							ranges.shed_row(0);
							if(ranges.n_rows==0)
								break;
						}
					}*/
						
				}
			}

		}

					
			
		/*count++;
		if(count>3)
		{
			int temp_laser;
			mat tempscan = zeros<mat>(1,721);
			fscanf(plaser,"%d;",&temp_laser);
			tempscan(0,0)=temp_laser;
			//cout<<"get time "<<tempscan(0,0)<<endl;
			for(int i=0;i<1081; i++)
			{
				fscanf(plaser,"%d;",&temp_laser);
				if(i>=180 && i<900)
				{
					tempscan(0,i-179)=temp_laser;
				}
			}
			
			fscanf(plaser,"\n");
			ranges.insert_rows(ranges.n_rows, tempscan);
			count=0;
		}*/
	

		/////////////////////////////////////////////////
		
		
		loop_rate.sleep();
		ros::spinOnce();
		
	}
	fclose(pf);
	fclose(plaser);
	ros::shutdown();
	return 0;
};

