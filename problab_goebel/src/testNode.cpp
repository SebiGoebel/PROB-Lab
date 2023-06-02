#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

struct Odom{
    double x;
    double y;
    double th;
};

class testClass{
    private:
        Odom odom_;
    public:
        void callback(const nav_msgs::Odometry::ConstPtr &msg);
        Odom getOdom() const {
            return this->odom_;
        }
};


void testClass::callback(const nav_msgs::Odometry::ConstPtr &msg){
    Odom o1;
    o1.x = msg->pose.pose.position.x;
    o1.y = msg->pose.pose.position.y;
    o1.th = tf::getYaw(msg->pose.pose.orientation);

    std::cout << "Seq: " << msg->header.seq << std::endl;
    std::cout << "Pose: X:" << o1.x << " Y:" << o1.y << " TH: " << o1.th << std::endl;
    std::cout << "---" << std::endl;

    this->odom_ = o1;
    
    std::cout << this->odom_.x << std::endl;
    std::cout << "---" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testNode");
    ros::NodeHandle n;
    testClass test;

    ros::Subscriber sub = n.subscribe("odom", 1000, &testClass::callback, &test);

    
    ros::Rate loop_rate(10);

    while(n.ok())
	{

		ros::spinOnce();               // check for incoming messages 
			
		std::cout << "ultimativer test: " << test.getOdom().x << std::endl;

    	loop_rate.sleep();
   	}

    return 0;
}