#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>

//command für das publishen eines goals
//rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 3.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_pub");
    ros::NodeHandle n;

    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    ros::Rate loop_rate(10); // publishen mit 10 Hz

    double goals [4][3] ={{3,0.5,M_PI/2},{3,4.5,M_PI},{1,4.5,-M_PI/2},{1,2,-M_PI/2}};

    // umrechnung in quaternionen //roll, pitch == 0
    //double qx = 0;
    //double qy = 0;
    //double qz = sin(yaw/2);
    //double qw = cos(yaw/2);

    //int seq = 0; // brauch ich nicht (nur vollständigkeitshalber)

    int goal_target = 3;


    while (ros::ok())
    {
        //seq++; // brauch ich nicht (nur vollständigkeitshalber)
        geometry_msgs::PoseStamped msg;

        //header inputs
        //msg.header.seq = seq; // brauch ich nicht (nur vollständigkeitshalber)
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        //initialising
        msg.pose = geometry_msgs::Pose();
        msg.pose.orientation = geometry_msgs::Quaternion();

        //pose
        msg.pose.position.x = goals[goal_target][0];
        msg.pose.position.y = goals[goal_target][1];
        msg.pose.position.z = 0;

        //orientation
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = sin(goals[goal_target][2]/2);
        msg.pose.orientation.w = cos(goals[goal_target][2]/2);

        
        // Ausgabe des goals
        double th_grad = goals[goal_target][2] / M_PI * 180;
        ROS_INFO("X: %f, Y: %f, TH(grad): %f", msg.pose.position.x, msg.pose.position.y, th_grad);

        goal_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}