#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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
    //code
};

// ========================================== Filter-CLASS ==========================================

class Filter
{
public:
    // Konstruktoren

    // Methoden
    void predict();
    void correct();
    Sample sample_motion_model(double v, double w, double x, double y, double th, double dt);
    void likelihood_field_range_finder_model();
    void resampling();

    void algorithmMCL();

    // setter + getter
    void setX_odom(double x)
    {
        this->x_odom = x;
    }

    double getX_odom() const
    {
        return x_odom;
    }

    void setY_odom(double y)
    {
        this->y_odom = y;
    }

    double getY_odom() const
    {
        return y_odom;
    }

    void setTh_odom(double th)
    {
        this->th_odom = th;
    }

    double getTh_odom() const
    {
        return th_odom;
    }

    void setV_cmd_vel(double v)
    {
        this->v_cmd_vel = v;
    }

    double getV_cmd_vel() const
    {
        return v_cmd_vel;
    }

    void setW_cmd_vel(double w)
    {
        this->w_cmd_vel = w;
    }

    double getW_cmd_vel() const
    {
        return w_cmd_vel;
    }

private:
    double x_odom = 0.0;
    double y_odom = 0.0;
    double th_odom = 0.0;
    double v_cmd_vel = 0.0;
    double w_cmd_vel = 0.0;

    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_th = 0.0;

    Sample samples[100];
    U_t motion_model;
    Z_t sensor_model;
};

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

void Filter::likelihood_field_range_finder_model()
{
    // code
}

void Filter::resampling()
{
    // code
}

void Filter::algorithmMCL(){
    //code
}




// ========================================== cmd_vel listener ==========================================

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


    Filter filter;

    filter.setV_cmd_vel(linear_x);
    filter.setW_cmd_vel(angular_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");

    ros::NodeHandle n;

    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, cmd_velCallback);
    //ros::Subscriber sub_odom = n.subscribe("odom", 1000, chatterCallback);

    ros::spin();

    return 0;
}
