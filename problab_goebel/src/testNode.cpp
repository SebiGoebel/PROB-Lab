#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testNode");
    ros::NodeHandle nh;

    // Publisher für die Ziel-Pose
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    ros::Rate loop_rate(10); // publishen mit 10 Hz

    // Erstellen der Ziel-Pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 3.0;
    goal_pose.pose.position.y = 0.5;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    while (ros::ok())
    {
        // Veröffentlichen der Ziel-Pose
        goal_pub.publish(goal_pose);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
