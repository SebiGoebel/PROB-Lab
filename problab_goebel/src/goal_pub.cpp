#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h> // für Odom
#include <tf/transform_broadcaster.h> // für tf::getYaw()

// command für das publishen eines goals
// rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 3.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'

float th;
float x;
float y;
float toleranz = 0.1;
int anzGoals = 4;
int wieOftGepublished = 5; // gibt an wie oft ein goal gepublished werden soll
                           // mindestens 2 Mal --> just to be save 5 Mal

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    th = tf::getYaw(odom->pose.pose.orientation);
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_pub");
    ros::NodeHandle n;

    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Subscriber sub = n.subscribe("odom", 100, odom_callback);

    ros::Rate loop_rate(10); // publishen mit 10 Hz

    float goals[anzGoals][3] = {
            //{3, 0.5,  M_PI / 2},
            {3, 1,  M_PI / 2},
            //{3, 4,  M_PI},
            {3, 4,  M_PI / 2},
            {1, 4, -M_PI / 2},
            {1,   2, -M_PI / 2}
        };

    int goal_target = 0;

    geometry_msgs::PoseStamped msg;

    // header inputs
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    // initialisieren
    msg.pose = geometry_msgs::Pose();
    msg.pose.orientation = geometry_msgs::Quaternion();

    int counter = 0;

    while (ros::ok())
    {
        // pose
        msg.pose.position.x = goals[goal_target][0];
        msg.pose.position.y = goals[goal_target][1];
        msg.pose.position.z = 0;

        // orientation
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = sin(goals[goal_target][2] / 2);
        msg.pose.orientation.w = cos(goals[goal_target][2] / 2);

        // Ausgabe des goals
        float th_grad = goals[goal_target][2] / M_PI * 180;
        ROS_INFO("Goal %d: X: %f, Y: %f, TH(grad): %f", goal_target, msg.pose.position.x, msg.pose.position.y, th_grad);

        // damit nicht die ganze zeit gepublished wird
        if(counter < wieOftGepublished){
            goal_pub.publish(msg);
            std::cout << "Counter: " << counter << std::endl;
            counter++;
        }
        
        // entfernung zum goal
        float dX = goals[goal_target][0] - x;
        float dY = goals[goal_target][1] - y;
        float dTh = goals[goal_target][2] - th;

        // wenn in toleranzbereich --> nächstes goal
        if (-toleranz < dX && dX < toleranz)
        {
            if (-toleranz < dY && dY < toleranz)
            {
                if ((-toleranz * 3) < dTh && dTh < (toleranz * 3))
                {
                    goal_target++;
                    counter = 0;
                    if (goal_target > (anzGoals-1))
                    {
                        goal_target = (anzGoals-1);
                        std::cout << "\n\nlast goal reached !!!\n" << std::endl;
                        return 0;
                    }
                }
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}