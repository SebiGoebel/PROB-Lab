#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char **argv)
{
    // Initialisiere den ROS-Node
    ros::init(argc, argv, "test2Node");
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
        pose_array.header.stamp = ros::Time::now();
        pub.publish(pose_array);
        rate.sleep();
    }

    return 0;
}
