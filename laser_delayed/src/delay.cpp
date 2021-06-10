#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

int filter_size=80;
int count=0;
sensor_msgs::LaserScan scan[80];
ros::Publisher odompub;

void laserCallback(const sensor_msgs::LaserScan msg)
{

if (count<filter_size)
	{
	scan[count]=msg;
	}

else
	{
	odompub.publish(scan[0]);
	for (int i=0;i<filter_size-1;i++)
		{	
		scan[i]=scan[i+1];	
		}
	scan[filter_size-1]=msg;
	}
count++;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, laserCallback);
  odompub=n.advertise<sensor_msgs::LaserScan>("odomscan", 1000);
  ros::spin();

  return 0;
}
