#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_pub_v2");

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);

    while(!client.waitForServer(ros::Duration(5.0))){
        std::cout << "Waiting for action sever!!!" << std::endl;
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    float goals[4][3] = {{3, 0.5, M_PI / 2}, {3, 4.5, M_PI}, {1, 4.5, -M_PI / 2}, {1, 2, -M_PI / 2}};

    for(int goal_target = 0; goal_target < 4; goal_target++){

        // pose
        goal.target_pose.pose.position.x = goals[goal_target][0];
        goal.target_pose.pose.position.y = goals[goal_target][1];
        goal.target_pose.pose.position.z = 0;

        // orientation
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = sin(goals[goal_target][2] / 2);
        goal.target_pose.pose.orientation.w = cos(goals[goal_target][2] / 2);

        // sending goal
        client.sendGoal(goal);

        // waiting for result
        client.waitForResult();
        
        ros::Duration(2.0).sleep();
    }
    return 0;
}