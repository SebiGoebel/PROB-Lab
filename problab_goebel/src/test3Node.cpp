#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

double dt = 0.0;
double time_stamp_old = 0.0;

double convertingTime2Double(ros::Time time){
    double time_in_double = time.toSec(); // converting ros::Time to a double value in seconds
    return time_in_double;
}

int main(int argc, char **argv)
{
    // Initialisiere den ROS-Node
    ros::init(argc, argv, "test3Node");
    ros::NodeHandle nh;

    // Erstelle einen Publisher für das Topic "particlecloud"
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("/particlecloud", 10);

    // Warte, bis sich der Publisher verbunden hat
    ros::Duration(1.0).sleep();

    // Erstelle ein PoseArray-Objekt
    geometry_msgs::PoseArray pose_array;

    // Füge drei vordefinierte Pose-Objekte hinzu
    geometry_msgs::Pose pose1;
    pose1.position.x = 1.0;
    pose1.position.y = 2.0;
    pose1.position.z = 0.0;

    geometry_msgs::Pose pose2;
    pose2.position.x = -1.0;
    pose2.position.y = 0.0;
    pose2.position.z = 0.0;

    geometry_msgs::Pose pose3;
    pose3.position.x = 3.0;
    pose3.position.y = -2.0;
    pose3.position.z = 0.0;

    pose_array.header.frame_id = "map"; // Setze den Frame des PoseArrays
    pose_array.poses.push_back(pose1);
    pose_array.poses.push_back(pose2);
    pose_array.poses.push_back(pose3);

    // test print --> erfolgreich
    /*
    for(int i = 0; i < 3; i++){
        std::cout << "TestObject " << i << std::endl;
        std::cout << pose_array.poses[i].position.x << std::endl;
    }
    */

    // Veröffentliche das PoseArray
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        ros::Time time = ros::Time::now();
        pose_array.header.stamp = time;

        dt = convertingTime2Double(time) - time_stamp_old;
        time_stamp_old = convertingTime2Double(time);

        std::cout << "            Dt: " << dt << std::endl;
        std::cout << "Time stamp old: " << time_stamp_old << std::endl;


        pub.publish(pose_array);
        rate.sleep();
    }

    return 0;
}
