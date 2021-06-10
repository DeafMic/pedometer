/*Condiguration: x, y and theta of body frame are measured through the IMU located in the belt -- Human Odometry (body);
 * In this node: pose_x, pose_y and body_z
x, y of laser frame are the same of body frame (plus a constant offset);
 In this node, pose_x+offset, pose_y+offset
theta of laser frame is measured using the compass located on the helmet -- Human Odometry(head).
 In this node, pose_z
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
//#include <imu_pedometer/imu.h>


ros::Publisher scan_pub;
float pose_x=0, pose_y=0, pose_z=2.0;
float pose_x_saved=0, pose_y_saved=0;
float pose_x_saved2=0, pose_y_saved2=0;
float head_x=0, head_y=0, body_z=2.0;
float body_x=0, body_y=0, head_z=0.0;
bool ReceivedHeadPose=false;
int timer=0;
int count = 0;

//
//void imuCallback(const imu_pedometer::imu::ConstPtr& msg) {
// if ((msg->tstamp > 2143.0) & (msg->tstamp < 2180.0))
// {
// timer=1;
// }
// else if ((msg->tstamp > 2248.0) & (msg->tstamp < 2293.0))
// {
// timer=3;
// }
// else if ((msg->tstamp > 2180.0) & (msg->tstamp < 2196.0))
// {
// timer=2;
// }

// else
// {
// timer=0;
// }
//}

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg){   //The human body odometry, the imu data from the body
/* if (timer==0)
 {pose_x=msg->linear.x;
 pose_y=msg->linear.y;}
else if (timer==1)
 {pose_x=1.4*msg->linear.x;
 pose_y=1.4*msg->linear.y;}
else if (timer==2)
 {pose_x=4.8*msg->linear.x;
 pose_y=4.8*msg->linear.y;}
else if (timer==3)
 {pose_x=1.6*msg->linear.x;
 pose_y=1.6*msg->linear.y;}*/
    body_x=msg->linear.x;     //The step size is set as 0.6, can be tested with *0.8/0.6
    body_y=msg->linear.y;
    body_z=msg->angular.z;
}

void headCallback(const geometry_msgs::Twist::ConstPtr& msg){   //The imu data from the head
    pose_x=msg->linear.x;
    pose_y=msg->linear.y;
    pose_z=msg->angular.z;
    ReceivedHeadPose = true;
}

void laserCallback(sensor_msgs::LaserScan msg){  //The laser data, publish the laser data to mapper
    count++;
    sensor_msgs::LaserScan scan;
    scan = msg;
    scan.header.stamp = ros::Time::now();
    scan_pub.publish(scan);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
    //Publish the laser scan data to topic /base_scan2
    scan_pub = node.advertise<sensor_msgs::LaserScan>("/base_scan", 50);
    //Subscriber to body imu, head imu and laser scanner
    ros::Subscriber sub = node.subscribe("/body_pose", 50, &poseCallback);
    ros::Subscriber sub2 = node.subscribe("/odomscan", 50, &laserCallback);
    ros::Subscriber sub3 = node.subscribe("/head_pose",50, &headCallback);
//    ros::Subscriber sub4 = node.subscribe("/imudata",50, &imuCallback);
    ros::Rate loop_rate(125);

    float pose_x_back=0, pose_y_back=0, pose_z_back=2.0;
    float diff_x=0, diff_y=0, diff_z=0.0;
    float diff_x2=0, diff_y2=0;
    float diff_x_back=0, diff_y_back=0, diff_z_back=0;
    float pose_x_back2=0, pose_y_back2=0;

    while (ros::ok())
    {
        ros::spinOnce();

       /* pose_x_saved = pose_x;   //Pose of the body frame
        pose_y_saved = pose_y;

        diff_x = pose_x - pose_x_back;   //x and y are calculated in the body frame
        diff_y = pose_y - pose_y_back;
        diff_z = pose_z - pose_z_back;  //heading z is calculated in the head frame

        diff_x2 = pose_x - pose_x_back2;
        diff_y2 = pose_y - pose_y_back2;

        std::cout<< "diff_x: " << diff_x << std::endl << std::endl;
        std::cout<< "diff_x2: " <<  diff_x2  <<  std::endl << std::endl;

//        ROS_INFO("diff_x: %.2f",diff_x);
//        ROS_INFO("diff_x2: %.2f",diff_x2);

        std::cout << "pose_x: " << pose_x << " pose_x_saved: " << pose_x_saved << " pose_x_back: " << pose_x_back << " pose_x_back2: " << pose_x_back2 << std::endl << std::endl;

//        ROS_INFO("pose_x: %.2f, pose_x_saved: %.2f, pose_x_back: %.2%f, pose_x_back2: %.2f", pose_x,pose_x_saved,pose_x_back,pose_x_back2);
        //The range of the heading is [-pi,pi]
        if (diff_z>3.14){
            diff_z = diff_z-6.28;
        }
        if (diff_z<-3.14){
            diff_z = diff_z+6.28;
        }

        std::cout << diff_z << std::endl;
//        ROS_INFO("diff_z: %.2f", diff_z);

        if(ReceivedHeadPose)
        {
            if(fabs(diff_z) > 0.6){   //?
                diff_z = diff_z/300;
                pose_z = pose_z_back + diff_z;
            }
            else if(fabs(diff_z) > 0.2){
                diff_z = diff_z/30;
                pose_z = pose_z_back + diff_z;
            }

            if(fabs(diff_x) > 0.6)
                pose_x = pose_x_back2;
            else{
                pose_x = pose_x_back2 + diff_x;
                diff_x_back = diff_x;
            }

            if (fabs(diff_y)>0.6)
                pose_y = pose_y_back2;
            else{
                pose_y = pose_y_back2 + diff_y;
                diff_y_back=diff_y;
            }
        }

        int test= -1 +rand()%3;   //???????    rand()%give give 0,1 or 2 randomly, test will be -1, 0 or 1
        std::cout << test << std::endl << std::endl;
//        ROS_INFO("test: %d",test);
        //If didn't receive the head pose
        if (!ReceivedHeadPose) {

            diff_x = diff_x_back;
            diff_y = diff_y_back;
            diff_z = diff_z_back;

            pose_x = pose_x_back2 + diff_x/10;
            pose_y = pose_y_back2 + diff_y/10;
            pose_z = pose_z + float(test)/20.0;    //test/20 will be -0.05 0 or 0.05
        }

        std::cout << "x: " << pose_x << std::endl;
        std::cout << "y: " << pose_y << std::endl;
        std::cout << "ang: " << pose_z << std::endl << std::endl; */

        static tf::TransformBroadcaster br;

        //Tranform between the odometry and the laser
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose_x, pose_y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, pose_z);//body_z);//pose_z);
        transform.setRotation(q);
        //t2 is the transform of theta between the body and head frame(The heading in the head frame gives the heading of the laser frame)
        tf::Transform t2;
        t2.setOrigin(tf::Vector3(0.0, 0.0, body_z-pose_z));
        tf::Quaternion q2;
        q2.setRPY(0.0, 0.0, 0.0);
        t2.setRotation(q2);

        //base_link is the body
        br.sendTransform(tf::StampedTransform(t2, ros::Time::now(), "laser", "base_link"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odometry_offset", "laser"));

       /* //pose_back is the data get directly from the subscriber
        //x[t-1], y[t-1]
        pose_x_back = pose_x_saved;
        pose_y_back = pose_y_saved;

        //x_corrc[t-1], y_correc[t-1]
        pose_x_back2 = pose_x;
        pose_y_back2 = pose_y;

        //z[t-1], after correction
        pose_z_back = pose_z;

        diff_z_back=diff_z;

        ReceivedHeadPose=false;*/
        loop_rate.sleep();

    }
    return 0;
};
