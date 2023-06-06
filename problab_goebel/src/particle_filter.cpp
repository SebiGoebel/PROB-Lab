#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <random>
#include <cmath>

//Hyperparameter
#define anzSamples 100
#define alpha_1 1
#define alpha_2 1
#define alpha_3 1
#define alpha_4 1
#define alpha_5 1
#define alpha_6 1

#define normalTriangularDistribution true // decides which distribution should be taken
                                          // [true --> normal distribution; false --> triangular distribution]

#define initialX 0.5
#define initialY 0.5
#define initialTH 0.0



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
    double laserScan[360];
};

struct Odom{
    double x;
    double y;
    double th;
};

// ================= sampleToPose =================

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
    void predict();
    void correct();
    //Sample sample_motion_model(double v, double w, double x, double y, double th, double dt);
    Sample sample_motion_model_Structs(U_t u_t, Odom odom, double dt);
    Sample sample_motion_model_this();
    void likelihood_field_range_finder_model();
    void resampling();

    void algorithmMCL();

    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);

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

    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_th_ = 0.0;

    Sample samples_old_[anzSamples];
    //Sample samples_predicted_[anzSamples];
    //Sample samples_corrected_[anzSamples];
    U_t motion_model_;
    Z_t sensor_model_;
    Odom odom_;
    double time_stamp_old_;
    double dt_;
};

Filter::Filter(){
    Odom o_default;
    o_default.x = initialX;
    o_default.y = initialY;
    o_default.th = initialTH;
    this->odom_ = o_default;

    this->pose_x_ = initialX;
    this->pose_y_ = initialY;
    this->pose_th_ = initialTH;

    U_t motion_model_default;
    motion_model_default.v = 0.0;
    motion_model_default.w = 0.0;
    this->motion_model_ = motion_model_default;

    Sample sample_default;
    sample_default.x = 0.0;
    sample_default.y = 0.0;
    sample_default.th = 0.0;
    sample_default.weight = 0.0;

    for(int i = 0; i < anzSamples; i++){
        this->samples_old_[i] = sample_default;
        //this->samples_predicted_[i] = sample_default;
        //this->samples_corrected_[i] = sample_default;
    }

    for(int i = 0; i < anzSamples; i++){
        this->sensor_model_.laserScan[i] = 0.0;
    }

    this->time_stamp_old_ = 0.0;
    this->dt_ = 0.0;

    std::cout << "initials set" << std::endl;
}

void Filter::predict()
{
    for(int i = 0; i < anzSamples; i++){
        //sampling
        //weighting
        //eintragen in neues sample array
    }
}

void Filter::correct()
{
    for(int i = 0; i < anzSamples; i++){
        //resampling
        //draw i with probability proportional to the weight
    }
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

Sample Filter::sample_motion_model_Structs(U_t u_t, Odom odom, double dt)
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
    double x_ = odom.x - ((v_hat/w_hat) * sin(odom.th)) + ((v_hat/w_hat) * sin(odom.th + w_hat * dt));
    double y_ = odom.y + ((v_hat/w_hat) * cos(odom.th)) - ((v_hat/w_hat) * sin(odom.th + w_hat * dt));
    double th_ = odom.th + w_hat * dt + th_hilf * dt;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    return s1;
}

Sample Filter::sample_motion_model_this()
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
    double x_ = this->odom_.x - ((v_hat/w_hat) * sin(this->odom_.th)) + ((v_hat/w_hat) * sin(this->odom_.th + w_hat * this->dt_));
    double y_ = this->odom_.y + ((v_hat/w_hat) * cos(this->odom_.th)) - ((v_hat/w_hat) * sin(this->odom_.th + w_hat * this->dt_));
    double th_ = this->odom_.th + w_hat * this->dt_ + th_hilf * this->dt_;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    return s1;
}

void Filter::likelihood_field_range_finder_model()
{
    // code
}

void Filter::resampling()
{
    // code
}

void Filter::algorithmMCL(){
    Sample samples[anzSamples];
    Sample samples_new[anzSamples];
    for(int i = 0; i < anzSamples; i++){
        //samples[i] = sample_motion_model_Structs(this->motion_model_, this->odom_, dt);
    }
}

void Filter::callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg){

    this->motion_model_.v = cmd_vel_msg->linear.x;
    this->motion_model_.w = cmd_vel_msg->angular.z;

    // test print
    //std::cout << "test  V: " << this->motion_model_.v << ", W: " << this->motion_model_.w << std::endl;
}

void Filter::callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg){

    this->odom_.th=tf::getYaw(odom_msg->pose.pose.orientation);
	this->odom_.x=odom_msg->pose.pose.position.x;
	this->odom_.y=odom_msg->pose.pose.position.y;
    
    // test print
    //std::cout << "X: " << this->odom_.x << ", Y: " << this->odom_.y << ", TH: " << this->odom_.th << std::endl;
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

    // publisher
    ros::Publisher pub_particle_cloud = n.advertise<geometry_msgs::PoseArray>("/particlecloud", 10);
    geometry_msgs::PoseArray sample_poses;
    sample_poses.header.frame_id = "map"; // setzen des fram_ids auf map

    // 3 hard coded test poses
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

    
    sample_poses.poses.push_back(pose1);
    sample_poses.poses.push_back(pose2);
    sample_poses.poses.push_back(pose3);

    ros::Rate loop_rate(10); // updating with 10 Hz
    double time = 0.0;

    while(n.ok())
	{
		ros::spinOnce();
        
        // setting time stamp
        ros::Time time = ros::Time::now();
        sample_poses.header.stamp = time;

        filter.updateDt(time);
        filter.setTime_stamp_old(convertingTime2Double(time));


        //test prints
        std::cout << "Time: " << filter.getDt() << std::endl;
        std::cout << "Final X: " << filter.getOdom().x << ", Y: " << filter.getOdom().y << ", TH: " << filter.getOdom().th << std::endl;
        std::cout << "      V: " << filter.getU_t().v << ", W: " << filter.getU_t().w << std::endl;

        //publishing sample poses
        pub_particle_cloud.publish(sample_poses);

    	loop_rate.sleep();
   	}


    return 0;
}
