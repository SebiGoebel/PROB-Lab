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
#include <nav_msgs/OccupancyGrid.h>
#include <random>
#include <cmath>
#include <algorithm>

int counter = 0;

//Hyperparameter

// ------------ Allgemeine Einstellungen ------------

#define anzSamples 25

#define nurMotionModel false // wenn true --> zeigt nur das ausgewählte Motion Model an
                            // ACHTUNG: wenn true --> es wird nicht resamplet

#define motionModelOdom false // decides which motionmodel should be taken
                             // [true --> Odom; false --> Motion Command]

#define normalTriangularDistribution true // decides which distribution should be taken
                                          // [true --> normal distribution; false --> triangular distribution]

#define initialPosesNormalverteilt true // bei alles sensor models die nur mit odom arbeiten
                                         // ist es notwendig die initial pose zu kennen
                                         // --> muss deshalb auf false gesetzt werden

#define sensorModelRangeFinder true // decides which sensormodel should be taken
                                    // [true --> range_finder; false --> Odom]

#define initialX 0.5
#define initialY 0.5
#define initialTH 0.0

// ------------ Motion Models ------------

//sample_motion_model_velocity
#define alpha_1 0.8
#define alpha_2 0.2
#define alpha_3 0.2
#define alpha_4 0.8
#define alpha_5 0.2
#define alpha_6 0.6

//sample_motion_model_odometry
#define alpha_odom_1 0.1
#define alpha_odom_2 0.1
#define alpha_odom_3 0.1
#define alpha_odom_4 0.1


// ------------ Sensor Models ------------

//laserScanner
#define laserScannerPositionX 0.0
#define laserScannerPositionY 0.0

/* AMCL Argumente
<remap from="scan"                      to="/scan"/>
    <param name="laser_max_range"           value="3.5"/>
    <param name="laser_max_beams"           value="180"/>
    <param name="laser_z_hit"               value="0.5"/>
    <param name="laser_z_short"             value="0.05"/>
    <param name="laser_z_max"               value="0.05"/>
    <param name="laser_z_rand"              value="0.5"/>
    <param name="laser_sigma_hit"           value="0.2"/>
    <param name="laser_lambda_short"        value="0.1"/>
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <param name="laser_model_type"          value="likelihood_field"/>
*/

#define zHit 0.5
#define zShort 0.05
#define zMax 10.0
#define zRand 0.01
#define sigmaHit 0.8

//Map
int map_height; // Ermittelt => 320
int map_width;  // Ermittelt => 320
#define sizeOfMap 320 // 320 * 320


// ================= random distributions =================

double random_value(double min, double max){
    // Zufallszahlengenerator
    std::random_device rd;
    std::mt19937 generator(rd());

    // Gleichverteilung zw. min- und max-value
    std::uniform_real_distribution<double> distribution(min, max);

    // Generiere eine Zufallszahl
    double randomVal = distribution(generator);

    return randomVal;
}

double sample_normal_distribution(double b){ // b -> variance

    double helpervar = 0.0;

    for(int i = 0; i < 12; i++){
        helpervar += random_value(-1.0, 1.0);
    }

    double result = b / 6 * helpervar;

    return result;
}

double sample_triangular_distribution(double b){ // b -> variance
    double helpervar1 = random_value(-1.0, 1.0);
    double helpervar2 = random_value(-1.0, 1.0);
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

double probabilityDist(double dist, double sigma){

    // ACHTUNG: simga = σ²
    
    double mean = 0.0;

    double exponent = -(pow(dist - mean, 2) / (2 * sigma));
    double prob = (1 / (sqrt(2 * M_PI * sigma))) * exp(exponent);

    return prob;
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
    double laserMaxRange;
    double laserMinRange;

    //double laserScan[360];
    std::vector<float> laserScan;
};

struct Odom{
    double x;
    double y;
    double th;

    double x_d1;
    double y_d1;
    double th_d1;

    double x_vel;
    //double y_vel;
    double th_vel;
};

struct RayCastingPoint{
    double x;
    double y;
    int mapWert;
};

struct Map{
    double resolution;
    int height;
    int width;
    RayCastingPoint ray_castingPoints_2D[sizeOfMap][sizeOfMap];
    std::vector<RayCastingPoint> ray_castingPoints; // vector für alle Zellenmittelpunkte
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
    geometry_msgs::Pose getSampleFromSample_old_anStelle(int i);
    
    //Motion Models
    Sample sample_motion_model_Structs(U_t u_t, Sample sample_old, double dt);
    Sample sample_motion_model_this(Sample sample_old);
    Sample sample_motion_model_odometry(Sample sample_old);
    
    //Sensor Models
    double likelihood_field_range_finder_model(Sample sample);
    double odom_vel_sensor_model_linearer_regler(Sample predictedSample, Sample oldSample);
    double odom_vel_sensor_model(Sample predictedSample, Sample oldSample);
    void updateOdomSample();
    double odom_vel_sensor_model_odomSample(Sample predictedSample);
    std::vector<Sample> low_variance_sampler(std::vector<Sample> predictedSamples);

    void algorithmMCL();

    // callback funktionen
    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void callback_odom_motion(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg);

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

    void setMap(const Map& map){
        this->map_ = map;
    }

    Map getMap() const {
        return this->map_;
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

    //wird erst gebraucht wenn ich saples_old_ zu einem vector mache
    void copyVector(const std::vector<int>& sourceVector) {
        //this->samples_old_ = sourceVector;
    }

private:
    // samples
    Sample samples_old_[anzSamples];
    Sample odomSample_;
    
    // models
    U_t motion_model_;
    Z_t sensor_model_;
    Odom odom_;
    Map map_;
    
    // time variables
    double time_stamp_old_;
    double dt_;
};

// ========================================== Konstruktor: setting initials ==========================================

Filter::Filter(){
    Odom o_default;
    o_default.x = initialX;
    o_default.y = initialY;
    o_default.th = initialTH;
    o_default.x_d1 = initialX;
    o_default.y_d1 = initialY;
    o_default.th_d1 = initialTH;
    o_default.x_vel = 0.0;
    //o_default.y_vel = 0.0;
    o_default.th_vel = 0.0;
    this->odom_ = o_default;

    U_t motion_model_default;
    motion_model_default.v = 0.0;
    motion_model_default.w = 0.0;
    this->motion_model_ = motion_model_default;

    if(initialPosesNormalverteilt){
        // Initial Posen: Normalverteilt
        //https://cplusplus.com/reference/random/normal_distribution/
        std::default_random_engine generator;
        std::normal_distribution<double> distributionPosition(0.5, 1.0); // µ = 0.5, σ = 1.0

        for(int i = 0; i < anzSamples; i++){
            this->samples_old_[i].x = distributionPosition(generator);
            this->samples_old_[i].y = distributionPosition(generator);
            //this->samples_old_[i].th = random_value(-M_PI, M_PI);
            this->samples_old_[i].th = random_value(-1.0, 1.0);
            this->samples_old_[i].weight = (double)(1.0/(double)anzSamples); // gewichtung gleichmäßig verteilt
        }
    }
    else
    {
        // Initial Posen: fixe Posen
        Sample sample_default;
        sample_default.x = initialX;
        sample_default.y = initialY;
        sample_default.th = initialTH;
        sample_default.weight = (double)(1.0/(double)anzSamples); // gewichtung gleichmäßig verteilt

        for(int i = 0; i < anzSamples; i++){
            this->samples_old_[i] = sample_default;
            //this->samples_predicted_[i] = sample_default;
            //this->samples_corrected_[i] = sample_default;
        }
    }

    Sample odomSample_default;
    odomSample_default.x = initialX;
    odomSample_default.y = initialY;
    odomSample_default.th = initialTH;
    this->odomSample_ = odomSample_default;

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

// ========================================== Motion Models ==========================================

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

    // kinematic model
    double x_ = sample_old.x - ((v_hat/w_hat) * sin(sample_old.th)) + ((v_hat/w_hat) * sin(sample_old.th + w_hat * this->dt_));
    double y_ = sample_old.y + ((v_hat/w_hat) * cos(sample_old.th)) - ((v_hat/w_hat) * cos(sample_old.th + w_hat * this->dt_));
    double th_ = sample_old.th + w_hat * this->dt_ + th_hilf * this->dt_;

    //std::cout << "test: x_: " << x_ << ", y_: " << y_ << ", th_: " << (th_*180/M_PI) << std::endl;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    //s1.weight = 0.1835;//(double)(1.0/(double)anzSamples); // gewichtung gleichmäßig verteilt
    return s1;
}

Sample Filter::sample_motion_model_odometry(Sample sample_old){

    // Buch Seite 110
    // δ == delta

    //double toleranz = 0.01;
    //if(this->odom_.x > (initialX + toleranz) || this->odom_.x < (initialX - toleranz) || this->odom_.y > (initialY + toleranz) || this->odom_.y < (initialY - toleranz) || this->odom_.th > (initialTH + toleranz) || this->odom_.th < (initialTH - toleranz)){
    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){

        double delta_rot_1 = atan2((this->odom_.y - this->odom_.y_d1), (this->odom_.x - this->odom_.x_d1)) - this->odom_.th_d1;
        double delta_trans = sqrt(pow((this->odom_.x_d1 - this->odom_.x), 2) + pow((this->odom_.y_d1 - this->odom_.y), 2));
        double delta_rot_2 = this->odom_.th - this->odom_.th_d1 - delta_rot_1;

        double variance1 = alpha_odom_1 * abs(delta_rot_1) + alpha_odom_2 * abs(delta_trans);
        double variance2 = alpha_odom_3 * abs(delta_trans) + alpha_odom_4 * (abs(delta_rot_1) + abs(delta_rot_2));
        double variance3 = alpha_odom_1 * abs(delta_rot_2) + alpha_odom_2 * abs(delta_trans);

        double delta_rot_1_hat = delta_rot_1 - sampling(variance1);
        double delta_trans_hat = delta_trans - sampling(variance2);
        double delta_rot_2_hat = delta_rot_2 - sampling(variance3);

        double x_ = sample_old.x + delta_trans_hat * cos(sample_old.th + delta_rot_1_hat);
        double y_ = sample_old.y + delta_trans_hat * sin(sample_old.th + delta_rot_1_hat);
        double th_ = sample_old.th + delta_rot_1_hat + delta_rot_2_hat;

        // speichern in Sample-struct und return
        Sample s1;
        s1.x = x_;
        s1.y = y_;
        s1.th = th_;
        return s1;
    }
    else
    {
        return sample_old;
    }
}

// ========================================== Sensor Models ==========================================

/* AMCL Argumente
<remap from="scan"                      to="/scan"/>
    <param name="laser_max_range"           value="3.5"/>
    <param name="laser_max_beams"           value="180"/>
    <param name="laser_z_hit"               value="0.5"/>
    <param name="laser_z_short"             value="0.05"/>
    <param name="laser_z_max"               value="0.05"/>
    <param name="laser_z_rand"              value="0.5"/>
    <param name="laser_sigma_hit"           value="0.2"/>
    <param name="laser_lambda_short"        value="0.1"/>
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <param name="laser_model_type"          value="likelihood_field"/>
*/

// für ein sample
double Filter::likelihood_field_range_finder_model(Sample sample)
{
    // wenn keine bewegung alles particles
    if(this->motion_model_.v < 0.1 && this->motion_model_.v > -0.1 && this->motion_model_.w < 0.1 && this->motion_model_.w > -0.1){
        // wenn keine bewegung --> alle particle gleich gewichten
        return (double)(1.0/(double)anzSamples);
    }

    double probability = 1.0;
    
    //double checkVar = sqrt(pow((this->map_.resolution/2),2) + pow((this->map_.resolution/2),2));

    for(int i = 0; i < this->sensor_model_.laserScan.size(); i++){ // für alle beams (i = angle of beam)
        if(this->sensor_model_.laserScan[i] >= this->sensor_model_.laserMinRange && this->sensor_model_.laserScan[i] < this->sensor_model_.laserMaxRange){ // check if beam is valid
            // Grad i umrechnen in rad
            double angleInGrad = double(i);
            double angleInRad = 0.0;
            if(i <= 180){
                angleInRad = angleInGrad / 180 * M_PI;
            }
            if(i > 180){
                angleInRad = (angleInGrad-360) / 180 * M_PI;
            }

            // Transforming Scanner to the world frame
            double x_ztk = sample.x + laserScannerPositionX * cos(sample.th) - laserScannerPositionY * sin(sample.th) + this->sensor_model_.laserScan[i] * cos(sample.th + angleInRad);
            double y_ztk = sample.y + laserScannerPositionY * cos(sample.th) + laserScannerPositionX * sin(sample.th) + this->sensor_model_.laserScan[i] * sin(sample.th + angleInRad);

            //std::cout << "sample_x: " << sample.x << ", sample_y: " << sample.y << " sample_th: " << sample.th << std::endl;
            //std::cout << "laser " << i << ": x_ztk: " << x_ztk << " ,y_ztk:" << y_ztk << ", Range: " << this->sensor_model_.laserScan[i] << ", angleInRad: " << angleInRad << std::endl;

            // vielleicht das th vom sensor (i oder angleInRad) berechnen --> float32 angle_increment  # angular distance between measurements [rad]
            // siehe Message definition:
            // http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

            // Zeile 7 --> dist²
            // start = maxDistanze = Mapdiagonale
            //double minDist = pow(this->map_.height * this->map_.resolution, 2) + pow(this->map_.width * this->map_.resolution, 2);
            double minDist_2 = this->sensor_model_.laserMaxRange;

            //int checkrayCastingPoint_index = 0;
            //int checkrayCastingPoint_index_X = 0;
            //int checkrayCastingPoint_index_Y = 0;

            for(int j = 0; j < this->map_.ray_castingPoints.size(); j++){
                // berechnen der euklidischen Distanz zu jedem validen Punkt (!= -1) --> dist²
                double dist_berechnet = pow((x_ztk - this->map_.ray_castingPoints[j].x), 2) + pow((y_ztk - this->map_.ray_castingPoints[j].y), 2);
                // übernehmen des Minimums
                if(dist_berechnet < minDist_2){
                    minDist_2 = dist_berechnet;

                    // nur fürs printen
                    //checkrayCastingPoint_index = j;
                }
            }

            //for(int zeile = 0; zeile < sizeOfMap; zeile++){
            //    for(int spalte = 0; spalte < sizeOfMap; spalte++){
            //        double dist_berechnet = sqrt(pow(x_ztk - this->map_.ray_castingPoints_2D[zeile][spalte].x, 2) + pow(y_ztk - this->map_.ray_castingPoints_2D[zeile][spalte].y, 2));
            //        if(dist_berechnet < minDist){
            //            minDist = dist_berechnet;
            //
            //            checkrayCastingPoint_index_X = spalte;
            //            checkrayCastingPoint_index_Y = zeile;
            //        }
            //    }
            //}

            // check ob mehr als diagonale
            //if(minDist > checkVar){
            //    counter++;
            //    std::cout << "ACHTUNG: minDist>" << checkVar << " !!! ";
            //}

            // test print für 2D-Array
            //std::cout << "Winkel: " << i << " minDist: " << minDist << ", x_ztk: " << x_ztk << ", y_ztk:" << y_ztk
            //<< ", rayCastPoint X: " << this->map_.ray_castingPoints_2D[checkrayCastingPoint_index_Y][checkrayCastingPoint_index_X].x << ", rayCastPoint Y: " << this->map_.ray_castingPoints_2D[checkrayCastingPoint_index_Y][checkrayCastingPoint_index_X].y << std::endl;
            
            // test print für vector
            //std::cout << "Winkel: " << i << " minDist: " << minDist << ", x_ztk: " << x_ztk << ", y_ztk:" << y_ztk
            //<< ", rayCastPoint X: " << this->map_.ray_castingPoints[checkrayCastingPoint_index].x << ", rayCastPoint Y: " << this->map_.ray_castingPoints[checkrayCastingPoint_index].y << " Mapwert: " << this->map_.ray_castingPoints[checkrayCastingPoint_index].mapWert << std::endl;

            // Zeile 8 --> berechnen von q (probability)
            probability = probability * (zHit * probabilityDist(minDist_2, sigmaHit) + (zRand / zMax));
        }
    }
    //std::cout << "Prob: " << probability << std::endl;
    //if(probability != 1){
    //    std::cout << "Counter: " << counter << "    checkerVar: " << checkVar << " wie viele laserScanner: " << counterCheck << std::endl;
    //    exit(0);
    //}
    return probability;
}

// ---------- odometrie sensor models ----------

double Filter::odom_vel_sensor_model_linearer_regler(Sample predictedSample, Sample oldSample){

    double gewichtung = 0.0;

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){
        //double gewichtung = (double)(1.0/(double)anzSamples); // starten mit allen Samples gleichgewichtet
        // Regelparameter nach Siegwart (2004)
        double kp = 0.4;
        double ka = 1;
        double kb = -0.3;
        
        // Differenzen zwischen old und predicted Sample
        double delta_X = predictedSample.x - oldSample.x;
        double delta_Y = predictedSample.y - oldSample.y;
        double delta_Th = predictedSample.th - oldSample.th;

        // berechnen von v und w --> mit lineare regelung
        double p = sqrt((delta_X * delta_X) + (delta_Y * delta_Y));
        double alpha = -oldSample.th + atan2(delta_Y, delta_X);
        
        // begrenzen von alpha
        if(alpha > M_PI){
            alpha = alpha - 2 * M_PI;
        }
        if(alpha < -M_PI){
            alpha = alpha + 2 * M_PI;
        }
        
        // berechnen und begrenzen von beta
        double beta = delta_Th - alpha;
        if(beta > M_PI){
            beta = beta - 2 * M_PI;
        }
        if(beta < -M_PI){
            beta = beta + 2 * M_PI;
        }

        // berechnnen de
        double v_regler = kp * p;
        double w_regler = ka * alpha + kb * beta;

        // Differenz zwischen odom_vels und berechneten vels
        double diffV = v_regler - this->odom_.x_vel;
        double diffW = w_regler - this->odom_.th_vel;

        gewichtung = abs(diffV) * abs(diffW);

        if(gewichtung != 0.0){
            gewichtung = 1.0 / gewichtung;
        }
    }
    else
    {
        // wenn keine bewegung --> alle particle gleich gewichten
        gewichtung = (double)(1.0/(double)anzSamples);
    }
    
    return gewichtung;
}

double Filter::odom_vel_sensor_model(Sample predictedSample, Sample oldSample){
    double gewichtung = 0.0;

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){

        double v_odom = this->odom_.x_vel;
        double w_odom = this->odom_.th_vel;

        // berechnen von x, y, th aus den odom v und w
        double x_berechnet = oldSample.x - ((v_odom/w_odom) * sin(oldSample.th)) + ((v_odom/w_odom) * sin(oldSample.th + w_odom * this->dt_));
        double y_berechnet = oldSample.y + ((v_odom/w_odom) * cos(oldSample.th)) - ((v_odom/w_odom) * cos(oldSample.th + w_odom * this->dt_));
        double th_berechnet = oldSample.th + w_odom * this->dt_;

        double diffX = predictedSample.x - x_berechnet;
        double diffY = predictedSample.y - y_berechnet;
        double diffTh = predictedSample.th - th_berechnet;

        gewichtung = abs(diffX) * abs(diffY) * abs(diffTh);

        if(gewichtung != 0.0){
            gewichtung = 1.0 / gewichtung;
        }
    }
    else
    {
        // wenn keine bewegung --> alle particle gleich gewichten
        gewichtung = (double)(1.0/(double)anzSamples);
    }

    return gewichtung;
}

void Filter::updateOdomSample(){
    double v_odom = this->odom_.x_vel;
    double w_odom = this->odom_.th_vel;

    if(v_odom != 0.0 &&  w_odom != 0.0){
        // berechnen von x, y, th aus den odom v und w
        this->odomSample_.x = this->odomSample_.x - ((v_odom/w_odom) * sin(this->odomSample_.th)) + ((v_odom/w_odom) * sin(this->odomSample_.th + w_odom * this->dt_));
        this->odomSample_.y = this->odomSample_.y + ((v_odom/w_odom) * cos(this->odomSample_.th)) - ((v_odom/w_odom) * cos(this->odomSample_.th + w_odom * this->dt_));
        this->odomSample_.th = this->odomSample_.th + w_odom * this->dt_;
    }
}

double Filter::odom_vel_sensor_model_odomSample(Sample predictedSample){
    double gewichtung = 0.0;

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){

        double diffX = predictedSample.x - this->odomSample_.x;
        double diffY = predictedSample.y - this->odomSample_.y;
        double diffTh = predictedSample.th - this->odomSample_.th;

        gewichtung = abs(diffX) * abs(diffY) * abs(diffTh);

        if(gewichtung != 0.0){
            gewichtung = 1.0 / gewichtung;
        }
    }
    else
    {
        // wenn keine bewegung --> alle particle gleich gewichten
        gewichtung = (double)(1.0/(double)anzSamples);
    }

    return gewichtung;
}

// ========================================== resampling ==========================================

std::vector<Sample> Filter::low_variance_sampler(std::vector<Sample> predictedSamples) {
    // empty set of samples
    std::vector<Sample> correctedSamples(anzSamples);

    // random value
    //double r = random_value(0.0, 1.0/anzSamples);
    double r = random_value(0.0, (double)(1.0/(double)anzSamples));

    // Gewichte Normalisieren
    std::vector<double> normWeights(anzSamples);
    double sumWeights = 0.0;
    for(int i = 0; i < anzSamples; i++){
        sumWeights += predictedSamples[i].weight;
    }
    if(sumWeights != 0.0){
        for(int i = 0; i < anzSamples; i++){
            normWeights[i] = predictedSamples[i].weight / sumWeights;
        }
    }

    double c = normWeights[0]; // cumulative sum
    int i = 0;                 // index for predictedSamples --> drawing
    int j = 0;                 // index for correctedSamples --> placing/speichern

    // draw with replacement
    for(int m = 0; m < anzSamples; m++){
        //double u = r + m * (1/anzSamples);
        double u = r + ((double) m * (double)(1.0/(double)anzSamples));
        while(u > c){
            i = i + 1;
            c = c + normWeights[i];
        }
        correctedSamples[j] = predictedSamples[i];
        j++;
    }

    return correctedSamples;
}

// ========================================== Monte Carlo Localization Algorithm ==========================================

void Filter::algorithmMCL(){
    std::vector<Sample> samples_predicted(anzSamples); // for sampling and weighting
    std::vector<Sample> samples_new(anzSamples);      // for resampling

    updateOdomSample();
    for(int i = 0; i < anzSamples; i++){
        if(motionModelOdom){
            samples_predicted[i] = sample_motion_model_odometry(this->samples_old_[i]);
        }
        else
        {
            samples_predicted[i] = sample_motion_model_this(this->samples_old_[i]);
        }
        if(sensorModelRangeFinder){
            samples_predicted[i].weight = likelihood_field_range_finder_model(samples_predicted[i]);
        }
        else
        {
            samples_predicted[i].weight = odom_vel_sensor_model_odomSample(samples_predicted[i]);
        }
    }
    std::cout << "test" << std::endl;
/*
    updateOdomSample();
    for(int i = 0; i < anzSamples; i++){
        // vielleicht weighting in einem eigenen loop
        //samples_predicted[i].weight = odom_vel_sensor_model(samples_predicted[i], this->samples_old_[i]);
        samples_predicted[i].weight = odom_vel_sensor_model_odomSample(samples_predicted[i]);
    }
*/
    // Resampling
    samples_new = low_variance_sampler(samples_predicted);

    // Array übertragen an samples_old_
    for(int i = 0; i < anzSamples; i++){
        if(nurMotionModel){
            this->samples_old_[i] = samples_predicted[i]; // nur motion model
        }
        else
        {
            this->samples_old_[i] = samples_new[i]; // resampling wird übernommen
        }
    }

    // test print
    //std::cout << "samples_new size: " << samples_new.size() << std::endl;
    //std::cout << "sample " << testIndex << std::endl;
    //std::cout << "x: " << samples_new[testIndex].x << std::endl;
    //std::cout << "y: " << samples_new[testIndex].y << std::endl;
    //std::cout << "th: " << samples_new[testIndex].th << std::endl;
    //std::cout << "weight: " << samples_new[testIndex].weight << std::endl;
    //std::cout << "---" << std::endl;
}

// ========================================== callback funktionen ==========================================

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

    this->odom_.x_vel = odom_msg->twist.twist.linear.x;
    this->odom_.th_vel = odom_msg->twist.twist.angular.z;
    
    // test print
    //std::cout << "X: " << this->odom_.x << ", Y: " << this->odom_.y << ", TH: " << this->odom_.th << std::endl;
}

void Filter::callback_odom_motion(const nav_msgs::Odometry::ConstPtr& odom_msg){

    this->odom_.th_d1 = this->odom_.th;
    this->odom_.x_d1 = this->odom_.x;
    this->odom_.y_d1 = this->odom_.y;

    this->odom_.th = tf::getYaw(odom_msg->pose.pose.orientation);
	this->odom_.x = odom_msg->pose.pose.position.x;
	this->odom_.y = odom_msg->pose.pose.position.y;

    this->odom_.x_vel = odom_msg->twist.twist.linear.x;
    this->odom_.th_vel = odom_msg->twist.twist.angular.z;
    
    // test print
    //std::cout << "X: " << this->odom_.x << ", Y: " << this->odom_.y << ", TH: " << this->odom_.th << std::endl;
}

void Filter::callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	this->sensor_model_.laserMaxRange = laser_msg->range_max;
    this->sensor_model_.laserMinRange = laser_msg->range_min;
    this->sensor_model_.laserScan = laser_msg->ranges;
    
    // test print
    //std::cout << "laserMax: " << this->sensor_model_.laserMaxRange << std::endl;
    //for(int i = 0; i < this->sensor_model_.laserScan.size(); i++){
    //    std::cout << "range " << i << ": " << this->sensor_model_.laserScan[i] << std::endl;
    //}
    //std::cout << "size: " << this->sensor_model_.laserScan.size() << std::endl;
}

void Filter::callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg){

    
    this->map_.resolution = grid_msg->info.resolution;
    this->map_.height = grid_msg->info.height;
    this->map_.width = grid_msg->info.width;

    map_height = grid_msg->info.height;
    map_width = grid_msg->info.width;

    // Ursprung [0, 0] --> liegt im Mittelpunkt der Map --> [160, 160] = sizeOfMap / 2 = 8m / 0.05res
    // (8m = x-Richtung, 8m = -x-Richtung, 8m = y-Richtung, 8m = -y-Richtung)
    double ursprung_x = (this->map_.width * this->map_.resolution / 2);
    double ursprung_y = (this->map_.height * this->map_.resolution / 2);

    // kopieren des occupancy grids in mein 2D Array
    for(int zeile = 0; zeile < sizeOfMap; zeile++){
        for(int spalte = 0; spalte < sizeOfMap; spalte++){
            int index = zeile * sizeOfMap + spalte;
            
            //this->map_.data[zeile][spalte] = grid_msg->data[index];

            // befüllen des rayray_castingPoints für alle Zellenmittelpunkte wo nicht unbekannt
            //if(this->map_.data[zeile][spalte] != -1){
            //if(grid_msg->data[index] != -1 && grid_msg->data[index] != 0){ // --> bei einem && kommt ein Fehler !!!!!!!!!!!!!!
            if(grid_msg->data[index] != -1){
                if(grid_msg->data[index] != 0){
                    double x_value_spalte = spalte * this->map_.resolution - ursprung_x + (this->map_.resolution / 2);
                    double y_value_zeile  = zeile  * this->map_.resolution - ursprung_y + (this->map_.resolution / 2);

                    RayCastingPoint p1;
                    p1.x = x_value_spalte;
                    p1.y = y_value_zeile;
                    p1.mapWert = grid_msg->data[index];

                    this->map_.ray_castingPoints.push_back(p1);
                }
            }
            /*
            // für 2D Array
            double x_val = -ursprung_x + spalte * this->map_.resolution + (this->map_.resolution / 2);
            double y_val =  ursprung_y - zeile  * this->map_.resolution - (this->map_.resolution / 2);

            RayCastingPoint p2;
            p2.x = x_val;
            p2.y = y_val;
            p2.mapWert = grid_msg->data[index];

            this->map_.ray_castingPoints_2D[zeile][spalte] = p2;
            */
        }
    }

    //for(int zeile = 0; zeile < sizeOfMap; zeile++){
    //    for(int spalte = 0; spalte < sizeOfMap; spalte++){
    //        if(this->map_.ray_castingPoints_2D[zeile][spalte].x <= 2.28 && this->map_.ray_castingPoints_2D[zeile][spalte].x >= 2.27){
    //            std::cout << "Punkt: X: " << spalte << " Y: " << zeile << " X_KS: " << this->map_.ray_castingPoints_2D[zeile][spalte].x << " Y_KS: " << this->map_.ray_castingPoints_2D[zeile][spalte].y << " MapWert = " << this->map_.ray_castingPoints_2D[zeile][spalte].mapWert <<  std::endl;
    //        }
    //    }
    //}
    

    //for(int i = 0; i < this->map_.ray_castingPoints.size(); i++){
    //    if(this->map_.ray_castingPoints[i].x <= 2.23 && this->map_.ray_castingPoints[i].x >= 2.22){
    //        std::cout << i << " X: " << this->map_.ray_castingPoints[i].x << " Y: " << this->map_.ray_castingPoints[i].y << " MapWert = " << this->map_.ray_castingPoints[i].mapWert << std::endl;
    //    }
    //}
    //exit(0);
    
    //for(int i = 0; i < this->map_.ray_castingPoints.size(); i++){
    //    std::cout << "Punkt " << i << ": [" << this->map_.ray_castingPoints[i].x << ", " << this->map_.ray_castingPoints[i].y << "]" << std::endl;
    //}

    // test print
    //std::cout << "res: " << this->map_.resolution << ", height: " << this->map_.height << ", width: " << this->map_.width << std::endl;
}

// ========================================== main ==========================================

int main(int argc, char **argv)
{
    Filter filter;

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    // subscribers
    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, &Filter::callback_cmd_vel, &filter);
    //ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom, &filter);
    ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom_motion, &filter);
    ros::Subscriber sub_laser = n.subscribe("scan", 1000, &Filter::callback_laser, &filter);
    ros::Subscriber sub_occupancy_grid = n.subscribe("map", 1000, &Filter::callback_grid, &filter);
/*
    if(motionModelOdom){
        ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom, &filter);
        std::cout << "callback_odom is used" << std::endl;
    }
    else
    {
        ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom_motion, &filter);
        std::cout << "callback_odom_motion is used" << std::endl;
    }
*/

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
        std::cout << "Odom      X: " << filter.getOdom().x << ", Y: " << filter.getOdom().y << ", TH: " << filter.getOdom().th << std::endl;
        std::cout << "Odom Old  X: " << filter.getOdom().x_d1 << ", Y: " << filter.getOdom().y_d1 << ", TH: " << (filter.getOdom().th_d1*180/M_PI) << std::endl;
        std::cout << "Odom vels X: " << filter.getOdom().x_vel << ", TH: " << (filter.getOdom().th_vel*180/M_PI) << std::endl;
        std::cout << "          V: " << filter.getU_t().v << ", W: " << filter.getU_t().w << std::endl;
        std::cout << "laserMax: " << filter.getZ_t().laserMaxRange << std::endl;
        std::cout << "laserMin: " << filter.getZ_t().laserMinRange << std::endl;
        std::cout << "map_height: " << map_height << std::endl;
        std::cout << "map_width: " << map_width << std::endl;

        std::cout << "---" << std::endl;
/*
        for(int zeile = 0; zeile < sizeOfMap; zeile++){
            for(int spalte = 0; spalte < sizeOfMap; spalte++){
                std::cout << filter.getMap().data[zeile][spalte] << ", ";
            }
            std::cout << std::endl;
        }
*/
        std::cout << std::endl;

        //publishing sample poses
        pub_particle_cloud.publish(sample_poses);

    	loop_rate.sleep();
   	}


    return 0;
}