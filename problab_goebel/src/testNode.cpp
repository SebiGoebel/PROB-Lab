#include "ros/ros.h"
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");

    ros::NodeHandle nh;

    int anzSamples = 300;
    bool nurMotionModel = false;
    bool motionModelOdom = false;
    bool normalTriangularDistribution = true;
    bool initialPosesNormalverteilt = true;
    bool sensorModelRangeFinder = true;
    bool clearingPoses = true;
    double initialX = 0.5;
    double initialY = 0.5;
    double initialTH = 0.0;
    double alpha_1 = 0.2;
    double alpha_2 = 0.8;
    double alpha_3 = 0.2;
    double alpha_4 = 0.8;
    double alpha_5 = 0.4;
    double alpha_6 = 0.6;
    double alpha_odom_1 = 2.5;
    double alpha_odom_2 = 2.5;
    double alpha_odom_3 = 2.5;
    double alpha_odom_4 = 2.5;
    double laserScannerPositionX = 0.0;
    double laserScannerPositionY = 0.0;
    int jederWieVielteBeam = 20;
    double zHit = 0.5;
    double zShort = 0.05;
    double zMax = 3.5;
    double zRand = 0.01;
    double sigmaHit = 0.2;

    nh.getParam("anzSamples", anzSamples);
    nh.getParam("nurMotionModel", nurMotionModel);
    nh.getParam("motionModelOdom", motionModelOdom);
    nh.getParam("normalTriangularDistribution", normalTriangularDistribution);
    nh.getParam("initialPosesNormalverteilt", initialPosesNormalverteilt);
    nh.getParam("sensorModelRangeFinder", sensorModelRangeFinder);
    nh.getParam("clearingPoses", clearingPoses);
    nh.getParam("initialX", initialX);
    nh.getParam("initialY", initialY);
    nh.getParam("initialTH", initialTH);
    nh.getParam("alpha_1", alpha_1);
    nh.getParam("alpha_2", alpha_2);
    nh.getParam("alpha_3", alpha_3);
    nh.getParam("alpha_4", alpha_4);
    nh.getParam("alpha_5", alpha_5);
    nh.getParam("alpha_6", alpha_6);
    nh.getParam("alpha_odom_1", alpha_odom_1);
    nh.getParam("alpha_odom_2", alpha_odom_2);
    nh.getParam("alpha_odom_3", alpha_odom_3);
    nh.getParam("alpha_odom_4", alpha_odom_4);
    nh.getParam("laserScannerPositionX", laserScannerPositionX);
    nh.getParam("laserScannerPositionY", laserScannerPositionY);
    nh.getParam("jederWieVielteBeam", jederWieVielteBeam);
    nh.getParam("zHit", zHit);
    nh.getParam("zShort", zShort);
    nh.getParam("zMax", zMax);
    nh.getParam("zRand", zRand);
    nh.getParam("sigmaHit", sigmaHit);

    ROS_INFO("anzSamples = %d", anzSamples);
    ROS_INFO("nurMotionModel = %d", nurMotionModel);
    ROS_INFO("motionModelOdom = %d", motionModelOdom);
    ROS_INFO("normalTriangularDistribution = %d", normalTriangularDistribution);
    ROS_INFO("initialPosesNormalverteilt = %d", initialPosesNormalverteilt);
    ROS_INFO("sensorModelRangeFinder = %d", sensorModelRangeFinder);
    ROS_INFO("clearingPoses = %d", clearingPoses);
    ROS_INFO("initialX = %f", initialX);
    ROS_INFO("initialY = %f", initialY);
    ROS_INFO("initialTH = %f", initialTH);
    ROS_INFO("alpha_1 = %f", alpha_1);
    ROS_INFO("alpha_2 = %f", alpha_2);
    ROS_INFO("alpha_3 = %f", alpha_3);
    ROS_INFO("alpha_4 = %f", alpha_4);
    ROS_INFO("alpha_5 = %f", alpha_5);
    ROS_INFO("alpha_6 = %f", alpha_6);
    ROS_INFO("alpha_odom_1 = %f", alpha_odom_1);
    ROS_INFO("alpha_odom_2 = %f", alpha_odom_2);
    ROS_INFO("alpha_odom_3 = %f", alpha_odom_3);
    ROS_INFO("alpha_odom_4 = %f", alpha_odom_4);
    ROS_INFO("laserScannerPositionX = %f", laserScannerPositionX);
    ROS_INFO("laserScannerPositionY = %f", laserScannerPositionY);
    ROS_INFO("jederWieVielteBeam = %d", jederWieVielteBeam);
    ROS_INFO("zHit = %f", zHit);
    ROS_INFO("zShort = %f", zShort);
    ROS_INFO("zMax = %f", zMax);
    ROS_INFO("zRand = %f", zRand);
    ROS_INFO("sigmaHit = %f", sigmaHit);
    
    return 0;
}
