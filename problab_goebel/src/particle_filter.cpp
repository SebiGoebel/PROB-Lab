#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <cmath>

//Hyperparameter
#define anzSamples 100
#define alpha_1 0.8
#define alpha_2 0.2
#define alpha_3 0.2
#define alpha_4 0.8
#define alpha_5 0.2
#define alpha_6 0.6

#define normalTriangularDistribution true // decides which distribution should be taken
                                          // [true --> normal distribution; false --> triangular distribution]

#define initialX 0.5
#define initialY 0.5
#define initialTH 0.0

double laserMaxRange = 0.0;



// ================= random distributions =================

double random_value(){
    // Zufallszahlengenerator
    std::random_device rd;
    std::mt19937 generator(rd());

    // Bereich der Gleichverteilung
    double min = -1.0;
    double max = 1.0;

    // Gleichverteilung zw. -1 und 1
    std::uniform_real_distribution<double> distribution(min, max);

    // Generiere eine Zufallszahl
    double randomVal = distribution(generator);

    return randomVal;
}

double sample_normal_distribution(double b){ // b -> variance

    double helpervar = 0.0;

    for(int i = 0; i < 12; i++){
        helpervar += random_value();
    }

    double result = b / 6 * helpervar;

    return result;
}

double sample_triangular_distribution(double b){ // b -> variance
    double helpervar1 = random_value();
    double helpervar2 = random_value();
    return b * helpervar1 * helpervar2;
}

double sampling(double variance){
    double randNum;
    if(normalTriangularDistribution == true){
        randNum = sample_normal_distribution(variance);
    }
    if(normalTriangularDistribution == false){
        randNum = sample_triangular_distribution(variance);
    }
    return randNum;
}

// ================= STRUCTS =================

//Sample
struct Sample
{
    double x;
    double y;
    double th;
    double weight;
};

struct U_t{
    double v;
    double w;
};

struct Z_t{
    //double laserScan[360];
    std::vector<float> laserScan;
};

struct Odom{
    double x;
    double y;
    double th;
};

// ================= Sample --> geometry_msgs::Pose =================

geometry_msgs::Pose sample2Pose(Sample sample){
    geometry_msgs::Pose pose;

    // position
    pose.position.x = sample.x;
    pose.position.y = sample.y;
    pose.position.z = 0.0;

    // orientation
    //tf::Quaternion quaternions;
    //quaternions.setRPY(0, 0, sample.th);
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, sample.th);

    return pose;
}

// ================= ros::Time --> double =================

double convertingTime2Double(ros::Time time){
    double time_in_double = time.toSec(); // converting ros::Time to a double value in seconds
    return time_in_double;
}

// ========================================== Filter-CLASS ==========================================

class Filter
{
public:
    // Konstruktoren
    Filter();

    // Methoden
    //Sample sample_motion_model(double v, double w, double x, double y, double th, double dt);
    Sample sample_motion_model_Structs(U_t u_t, Sample sample_old, double dt);
    Sample sample_motion_model_this(Sample sample_old);
    geometry_msgs::Pose getSampleFromSample_old_anStelle(int i);
    double likelihood_field_range_finder_model();
    void resampling();

    void algorithmMCL();

    // callback funktionen
    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

    // setter + getter
    void setOdom(const Odom& odom){
        this->odom_ = odom;
    }

    Odom getOdom() const {
        return this->odom_;
    }

    void setU_t(const U_t& u_t){
        this->motion_model_ = u_t;
    }

    U_t getU_t() const {
        return this->motion_model_;
    }

    void setZ_t(const Z_t& z_t){
        this->sensor_model_ = z_t;
    }

    Z_t getZ_t() const {
        return this->sensor_model_;
    }

    void setTime_stamp_old(const double& time_stamp){
        this->time_stamp_old_ = time_stamp;
    }

    double getTime_stamp_old() const {
        return this->time_stamp_old_;
    }

    void setDt(const double& dt){
        this->dt_ = dt;
    }

    double getDt() const {
        return this->dt_;
    }

    void updateDt(ros::Time timeNow){
        this->dt_ = convertingTime2Double(timeNow) - this->time_stamp_old_;
    }

private:
    //double x_odom = 0.0;
    //double y_odom = 0.0;
    //double th_odom = 0.0;
    //double v_cmd_vel = 0.0;
    //double w_cmd_vel = 0.0;

    //double pose_x_ = 0.0;
    //double pose_y_ = 0.0;
    //double pose_th_ = 0.0;

    // samples
    Sample samples_old_[anzSamples];
    //Sample samples_predicted_with_motion_[anzSamples];
    //Sample samples_weighted_[anzSamples];
    
    // models
    U_t motion_model_;
    Z_t sensor_model_;
    Odom odom_;
    
    // time variables
    double time_stamp_old_;
    double dt_;
};

Filter::Filter(){
    Odom o_default;
    o_default.x = initialX;
    o_default.y = initialY;
    o_default.th = initialTH;
    this->odom_ = o_default;

    U_t motion_model_default;
    motion_model_default.v = 0.0;
    motion_model_default.w = 0.0;
    this->motion_model_ = motion_model_default;

    Sample sample_default;
    sample_default.x = initialX;
    sample_default.y = initialY;
    sample_default.th = initialTH;
    sample_default.weight = 1/anzSamples; // gewichtung gleichmäßig verteilt

    for(int i = 0; i < anzSamples; i++){
        this->samples_old_[i] = sample_default;
        //this->samples_predicted_[i] = sample_default;
        //this->samples_corrected_[i] = sample_default;
    }

    //for(int i = 0; i < anzSamples; i++){
    //    this->sensor_model_.laserScan.push_back(0.0);
    //}

    this->time_stamp_old_ = 0.0;
    this->dt_ = 0.0;

    std::cout << "initials set" << std::endl;
}

geometry_msgs::Pose Filter::getSampleFromSample_old_anStelle(int i) {
    return sample2Pose(this->samples_old_[i]);
}

/*
Sample Filter::sample_motion_model(double v, double w, double x, double y, double th, double dt)
{
    // preparing Input for sampling()
    double absV = abs(v);
    double absW = abs(w);

    double variance1 = alpha_1 * absV + alpha_2 * absW;
    double variance2 = alpha_3 * absV + alpha_4 * absW;
    double variance3 = alpha_5 * absV + alpha_6 * absW;
    
    // motion model
    double v_hat = v + sampling(variance1);
    double w_hat = w + sampling(variance2);
    double th_hilf = sampling(variance3);
    double x_ = x - ((v_hat/w_hat) * sin(th)) + ((v_hat/w_hat) * sin(th + w_hat * dt));
    double y_ = y + ((v_hat/w_hat) * cos(th)) - ((v_hat/w_hat) * sin(th + w_hat * dt));
    double th_ = th + w_hat * dt + th_hilf * dt;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    return s1;
}
*/

Sample Filter::sample_motion_model_Structs(U_t u_t, Sample sample_old, double dt)
{
    // preparing Input for sampling()
    double absV = abs(u_t.v);
    double absW = abs(u_t.w);

    double variance1 = alpha_1 * absV + alpha_2 * absW;
    double variance2 = alpha_3 * absV + alpha_4 * absW;
    double variance3 = alpha_5 * absV + alpha_6 * absW;
    
    // motion model
    double v_hat = u_t.v + sampling(variance1);
    double w_hat = u_t.w + sampling(variance2);
    double th_hilf = sampling(variance3);
    double x_ = sample_old.x - ((v_hat/w_hat) * sin(sample_old.th)) + ((v_hat/w_hat) * sin(sample_old.th + w_hat * dt));
    double y_ = sample_old.y + ((v_hat/w_hat) * cos(sample_old.th)) - ((v_hat/w_hat) * sin(sample_old.th + w_hat * dt));
    double th_ = sample_old.th + w_hat * dt + th_hilf * dt;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    return s1;
}

// für ein sample
Sample Filter::sample_motion_model_this(Sample sample_old)
{
    // preparing Input for sampling()
    double absV = abs(this->motion_model_.v);
    double absW = abs(this->motion_model_.w);

    double variance1 = alpha_1 * absV + alpha_2 * absW;
    double variance2 = alpha_3 * absV + alpha_4 * absW;
    double variance3 = alpha_5 * absV + alpha_6 * absW;
    
    // motion model
    double v_hat = this->motion_model_.v + sampling(variance1);
    double w_hat = this->motion_model_.w + sampling(variance2);
    double th_hilf = sampling(variance3);

    //std::cout << "---" << std::endl;
    //std::cout << "test: v_hat: " << v_hat << ", w_hat: " << w_hat << ", th_hilf: " << (th_hilf*180/M_PI) << std::endl;

    if(w_hat == 0.0 || v_hat == 0.0 || th_hilf == 0.0){
        return sample_old;
    }

    double x_ = sample_old.x - ((v_hat/w_hat) * sin(sample_old.th)) + ((v_hat/w_hat) * sin(sample_old.th + w_hat * this->dt_));
    double y_ = sample_old.y + ((v_hat/w_hat) * cos(sample_old.th)) - ((v_hat/w_hat) * cos(sample_old.th + w_hat * this->dt_));
    double th_ = sample_old.th + w_hat * this->dt_ + th_hilf * this->dt_;

    //std::cout << "test: x_: " << x_ << ", y_: " << y_ << ", th_: " << (th_*180/M_PI) << std::endl;
    

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    s1.weight = 0.0; // gewicht alle auf null setzen
    return s1;
}

// für ein sample
double Filter::likelihood_field_range_finder_model()
{
    double probability = 1.0;
    for(int i = 0; i < anzSamples; i++){
        //code
    }
    return 0;
}

void Filter::resampling()
{
    // code
}

void Filter::algorithmMCL(){
    
    Sample samples_predicted[anzSamples]; // for sampling and weighting
    //Sample samples_new[anzSamples]; // for resampling

    for(int i = 0; i < anzSamples; i++){
        samples_predicted[i] = sample_motion_model_this(this->samples_old_[i]);
        //samples[i].weight = likelihood_field_range_finder_model();
    }

    //for(int i = 0; i < anzSamples; i++){
        // vielleicht weighting in einem eigenen loop
    //}

    //for(int i = 0; i < anzSamples; i++){
        //resampling
    //}


    // Array übertragen an samples_old_
    for(int i = 0; i < anzSamples; i++){
        this->samples_old_[i] = samples_predicted[i];
    }
}



void Filter::callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg){

    this->motion_model_.v = cmd_vel_msg->linear.x;
    this->motion_model_.w = cmd_vel_msg->angular.z;

    // test print
    //std::cout << "test  V: " << this->motion_model_.v << ", W: " << this->motion_model_.w << std::endl;
}

void Filter::callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg){

    this->odom_.th = tf::getYaw(odom_msg->pose.pose.orientation);
	this->odom_.x = odom_msg->pose.pose.position.x;
	this->odom_.y = odom_msg->pose.pose.position.y;
    
    // test print
    //std::cout << "X: " << this->odom_.x << ", Y: " << this->odom_.y << ", TH: " << this->odom_.th << std::endl;
}

void Filter::callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	laserMaxRange = laser_msg->range_max;
    this->sensor_model_.laserScan = laser_msg->ranges;
    
    // test print
    //std::cout << "laserMax: " << laserMaxRange << std::endl;
    //for(int i = 0; i < this->sensor_model_.laserScan.size(); i++){
    //    std::cout << "range " << i << ": " << this->sensor_model_.laserScan[i] << std::endl;
    //}
    //std::cout << "size: " << this->sensor_model_.laserScan.size() << std::endl;
}

// ========================================== cmd_vel listener ==========================================
/*
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg)
{
    // linear
    double linear_x = cmd_vel_msg->linear.x; // x
    double linear_y = cmd_vel_msg->linear.y; // y --> sollte immer 0 sein
    double linear_z = cmd_vel_msg->linear.z; // --> immer 0

    // angular
    double angular_x = cmd_vel_msg->angular.x; // --> immer 0
    double angular_y = cmd_vel_msg->angular.y; // --> immer 0
    double angular_z = cmd_vel_msg->angular.z; // th

    //ROS_INFO("I heard linear:  [%f, %f, %f]", linear_x, linear_y, linear_z);
    //ROS_INFO("I heard angular: [%f, %f, %f]", angular_x, angular_y, angular_z);
    ROS_INFO("I heard motion command(u_t = (v w)^T): [%f, %f]", linear_x, angular_z);
}
*/

int main(int argc, char **argv)
{
    
    Filter filter;

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    

    // subscribers
    //ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, cmd_velCallback);
    //ros::Subscriber sub_odom = n.subscribe("odom", 1000, chatterCallback);
    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, &Filter::callback_cmd_vel, &filter);
    ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom, &filter);
    ros::Subscriber sub_laser = n.subscribe("scan", 1000, &Filter::callback_laser, &filter);

    // publisher
    ros::Publisher pub_particle_cloud = n.advertise<geometry_msgs::PoseArray>("/particlecloud", 10);
    
    // Messages
    geometry_msgs::PoseArray sample_poses; // poseArray for samples
    geometry_msgs::Pose pose; // pose for one sample

    // header infos
    sample_poses.header.frame_id = "map"; // setzen des fram_ids auf map


    ros::Rate loop_rate(10); // updating with 10 Hz
    double time = 0.0;

    while(n.ok())
	{
		ros::spinOnce();
        
        // setting time stamp
        ros::Time time = ros::Time::now();
        sample_poses.header.stamp = time;

        // calculating dt
        filter.updateDt(time);
        filter.setTime_stamp_old(convertingTime2Double(time));

        // clearing pose array
        sample_poses.poses.clear();

        // MCL Angorithm
        filter.algorithmMCL();

        // vom filter umwandeln in eine pose + eintragen in pose_array
        for(int i = 0; i < anzSamples; i++){
            pose = filter.getSampleFromSample_old_anStelle(i);
            sample_poses.poses.push_back(pose);
        }

        // test prints
        std::cout << "----------" << std::endl;
        std::cout << "Time (dt): " << filter.getDt() << std::endl;
        std::cout << "Odom X: " << filter.getOdom().x << ", Y: " << filter.getOdom().y << ", TH: " << (filter.getOdom().th*180/M_PI) << std::endl;
        std::cout << "     V: " << filter.getU_t().v << ", W: " << filter.getU_t().w << std::endl;

        //publishing sample poses
        pub_particle_cloud.publish(sample_poses);

    	loop_rate.sleep();
   	}


    return 0;
}
