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
#include <yaml-cpp/yaml.h>
#include <iostream>

// ================= STRUCTS =================

struct Hyperparameter{
    int anzSamples;
    // --- agemeine Einstellungen ---
    bool nurMotionModel;
    bool motionModelOdom;
    bool normalTriangularDistribution;
    bool initialPosesNormalverteilt;
    bool sensorModelRangeFinder;
    bool clearingPoses;
    double initialX;
    double initialY;
    double initialTH;
    // --- Motion Models ---
    //sample_motion_model_velocity
    double alpha_1;
    double alpha_2;
    double alpha_3;
    double alpha_4;
    double alpha_5;
    double alpha_6;
    //sample_motion_model_odometry
    double alpha_odom_1;
    double alpha_odom_2;
    double alpha_odom_3;
    double alpha_odom_4;
    // --- sensor Models ---
    //likelyhood_range_finder_model
    double laserScannerPositionX;
    double laserScannerPositionY;
    int jederWieVielteBeam;
    double zHit;
    double zShort;
    double zMax;
    double zRand;
    double sigmaHit;
};

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
    std::vector<float> laserScan;
};

struct Odom{
    // pose current time step
    double x;
    double y;
    double th;

    // pose last time step
    double x_d1;
    double y_d1;
    double th_d1;

    // velocities
    double x_vel; // => v
    double th_vel;// => ω
};

//Mittelpunkte der occupied cells
struct RayCastingPoint{
    double x;
    double y;
    int mapWert;
};

struct Map{
    double resolution;
    int height;
    int width;
    std::vector<RayCastingPoint> ray_castingPoints; // vector für alle Zellenmittelpunkte
};

// ================================================= FUNCTIONS =================================================

// ================= random distributions =================

//nach Pseudocode --> Thrun S.98 Table 5.4 (Table-beschreibung)
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

//nach Pseudocode --> Thrun S.98 Table 5.4 (Lines: 1 bis 2)
double sample_normal_distribution(double b){ // b -> variance
    double helpervar = 0.0;
    
    for(int i = 0; i < 12; i++){
        helpervar += random_value(-1.0, 1.0);
    }
    
    double result = b / 6 * helpervar;

    return result;
}

//nach Pseudocode --> Thrun S.98 Table 5.4 (Lines: 3 bis 4)
double sample_triangular_distribution(double b){ // b -> variance
    double helpervar1 = random_value(-1.0, 1.0);
    double helpervar2 = random_value(-1.0, 1.0);
    return b * helpervar1 * helpervar2;
}

// sampling Methode um entweder eine normal_distribution aufzurufen oder eine triangular distribution
double sampling(double variance, bool distribution){
    double randNum;
    if(distribution == true){
        randNum = sample_normal_distribution(variance);
    }
    if(distribution == false){
        randNum = sample_triangular_distribution(variance);
    }
    return randNum;
}

// Normalverteilung mit einstellbarer Varianz und abweichung x (dist) für range_finder
// Formel nach Equation 1 im Paper
double probabilityDist(double dist, double sigma){
    // ACHTUNG: simga = σ²
    double mean = 0.0;

    double exponent = -(pow(dist - mean, 2) / (2 * sigma));
    double prob = (1 / (sqrt(2 * M_PI * sigma))) * exp(exponent);

    return prob;
}

// ================= Converting Functions =================

// ------------ Sample --> geometry_msgs::Pose ------------

geometry_msgs::Pose sample2Pose(Sample sample){
    geometry_msgs::Pose pose;

    // position
    pose.position.x = sample.x;
    pose.position.y = sample.y;
    pose.position.z = 0.0;

    // orientation
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, sample.th);

    return pose;
}

// ------------ ros::Time --> double ------------

double convertingTime2Double(ros::Time time){
    double time_in_double = time.toSec(); // converting ros::Time to a double value in seconds
    return time_in_double;
}

// ========================================== Filter-CLASS ==========================================

class Filter
{
public:
    // Konstruktor
    /*
        The Filter konstruktor setzt initial werte auf wichtige variablen.
        Er übernimmt auch die parameter die von "param/hyperparameter.yaml" kommen
        Es setzt auch die initial posen am anfang:
            hier ist es möglich alle initial posen normal zu verteilen
            oder alle posen auf die initial werte des Roboters zu setzten [0.5, 0.5, 0]
        alle samples sind am anfang gleich gewichtet --> 1/M
    */
    Filter(Hyperparameter defaultParameters);

    // --- Methoden ---
    // Getter und Setter befinden sich innerhalb der Klasse
    // größere Methoden die auserhalb der Klasse geschrieben worden sind (--> mit Filter::Methode)
    geometry_msgs::Pose getSampleFromSample_old_anStelle(int i);
    
    // --------- Motion Models ---------
    /*
        das Motion Model wird mit: "motionModelOdom" gewählt [true --> Odom; false --> Motion Command]

        die folgenden 2 Methoden sind unterschiedliche motion models die den Pseudocodes von Thrun folgen
        sample_motion_model_motion_command --> verwendet den motion command um die samples zu bewegen -> (nach Thrun S.98)
        sample_motion_model_odometry       --> verwendet die Odometry um die samples zu bewegen       -> (nach Thrun S.110)

        leider gibt es beim Motion model Odometry zwei Probleme:
            - dieses Motion Model funktioniert nur jedes 3. Mal
            - bei einem funktionierenden Versuch bewegen sich die Samples zwar in die richtige Richtung, jedoch ist die Bewegung viel kleiner als die des Roboters

        da ich dieses Problm nicht lösen konnte habe ich das sample_motion_model_odometry auch nicht im Paper beschrieben,
        wollte es jedoch nicht aus meinem Code löschen falls ich mir das Problem im Sommer nochmal anschaue oder Sie Feedback für mich haben
        --> sample_motion_model_motion_command funktioniert einwandfrei
            (optimale Hyperparameter zum testen siehe Paper)
    */
    Sample sample_motion_model_motion_command(Sample sample_old); //nach Pseudocode Thrun S.98
    Sample sample_motion_model_odometry(Sample sample_old);       //nach Pseudocode Thrun S.110
    
    // --------- Sensor Models ---------
    /*
        das Sensor Model wird mit: "sensorModelRangeFinder" gewählt [true --> range_finder; false --> Odom (In paper -> appraoch #3)]

        die folgenden Methoden sind measurement models:
        - likelihood_field_range_finder_model verwendet den 2D laser scanner um die samples zu gewichten (nach Thrun S.143)
        - updateOdomSample und odom_vel_sensor_model_odomSample verwenden die Odometry um die samples zu gewichten (dabei handelt es sich um den approach #3)

        bei den anderen beiden models handelt es sich um approach #1 und #2:
        - odom_vel_sensor_model_linearer_regler
        - odom_vel_sensor_model

        die Probleme von den beiden Models sind im Paper beschrieben --> dadurch dass sie nur funktionieren wenn alle samples im robot origin [0.5, 0.5, 0] starten,
        habe ich sie im Code nur vorständigkeitshalber gelassen
        ACHTUNG: die approaches #1 und #2 kann man nicht mit den einstellungen von "param/hyperparameter.yaml" aufrufen !!!
    */
    double likelihood_field_range_finder_model(Sample sample);
    double odom_vel_sensor_model_linearer_regler(Sample predictedSample, Sample oldSample); // In paper --> odom approach #1 (works only when all samples spawn at the origin) (nach Thrun S.143)
    double odom_vel_sensor_model(Sample predictedSample, Sample oldSample);                 // In paper --> odom approach #2 (works only when all samples spawn at the origin)
    void updateOdomSample();                                                                // In paper --> odom approach #3
    double odom_vel_sensor_model_odomSample(Sample predictedSample);                        // In paper --> odom approach #3

    // --------- Resampler ---------
    /*
        bei low_variance_sampler hadelt es sich um den resampler nach Thrun S.86 im Kapitel 4.2.2 Importance Sampling
        Meine Implementierung folget dem Peudocode mit dem einzigen unterschied, dass ich in der Methode noch alle Gewichtungen normiere
    */
    std::vector<Sample> low_variance_sampler(std::vector<Sample> predictedSamples); // (nach Thrun S.86)

    // --------- Algorithm ---------
    /*
        Der Pariclefilter Algorithmus folgt dem Pseudocodes des MCLs nach Thrun S.200
        Diese Methode wird in der main aufgerufen
    */
    void algorithmMCL(); // nach Thrun S.200

    // --- callback funktionen ---
    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void callback_odom_motion(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg);

    // --- setter + getter ---
    void setHyperparameter(const Hyperparameter& params){
        this->params_ = params;
    }

    Hyperparameter getHyperparameter() const {
        return this->params_;
    }

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

private:
    // hyperparameters
    Hyperparameter params_;

    // samples
    std::vector<Sample> samples_old_;
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

Filter::Filter(Hyperparameter defaultParameters){

    this->params_ = defaultParameters;

    Odom o_default;
    o_default.x = this->params_.initialX;
    o_default.y = this->params_.initialY;
    o_default.th = this->params_.initialTH;
    o_default.x_d1 = this->params_.initialX;
    o_default.y_d1 = this->params_.initialY;
    o_default.th_d1 = this->params_.initialTH;
    o_default.x_vel = 0.0;
    o_default.th_vel = 0.0;
    this->odom_ = o_default;

    U_t motion_model_default;
    motion_model_default.v = 0.0;
    motion_model_default.w = 0.0;
    this->motion_model_ = motion_model_default;

    if(this->params_.initialPosesNormalverteilt){
        // Initial Posen: Normalverteilt
        //https://cplusplus.com/reference/random/normal_distribution/
        std::default_random_engine generator;
        std::normal_distribution<double> distributionPosition(0.5, 1.0); // µ = 0.5, σ = 1.0

        for(int i = 0; i < this->params_.anzSamples; i++){
            Sample initialSample;
            
            initialSample.x = distributionPosition(generator);
            initialSample.y = distributionPosition(generator);
            initialSample.th = random_value(-M_PI, M_PI);
            //initialSample.th = random_value(-1.0, 1.0);
            initialSample.weight = (double)(1.0/(double)this->params_.anzSamples); // gewichtung gleichmäßig verteilt

            this->samples_old_.push_back(initialSample);
        }
    }
    else
    {
        // Initial Posen: fixe Posen
        Sample sample_default;
        sample_default.x = this->params_.initialX;
        sample_default.y = this->params_.initialY;
        sample_default.th = this->params_.initialTH;
        sample_default.weight = (double)(1.0/(double)this->params_.anzSamples); // gewichtung gleichmäßig verteilt

        for(int i = 0; i < this->params_.anzSamples; i++){
            this->samples_old_.push_back(sample_default);
        }
    }

    Sample odomSample_default;
    odomSample_default.x = this->params_.initialX;
    odomSample_default.y = this->params_.initialY;
    odomSample_default.th = this->params_.initialTH;
    this->odomSample_ = odomSample_default;

    this->time_stamp_old_ = 0.0;
    this->dt_ = 0.0;

    std::cout << "initials set" << std::endl;
}

// ==================================================== Methoden ====================================================

geometry_msgs::Pose Filter::getSampleFromSample_old_anStelle(int i) {
    return sample2Pose(this->samples_old_[i]);
}

// ========================================== Motion Models ==========================================

// Motion Model für ein sample mit Motioncommand
// nach Pseudocode Thrun S.98
Sample Filter::sample_motion_model_motion_command(Sample sample_old)
{
    // preparing Input for sampling()
    double absV = abs(this->motion_model_.v);
    double absW = abs(this->motion_model_.w);

    double variance1 = this->params_.alpha_1 * absV + this->params_.alpha_2 * absW;
    double variance2 = this->params_.alpha_3 * absV + this->params_.alpha_4 * absW;
    double variance3 = this->params_.alpha_5 * absV + this->params_.alpha_6 * absW;
    
    // including randomness with sampling()
    double v_hat = this->motion_model_.v + sampling(variance1, this->params_.normalTriangularDistribution);
    double w_hat = this->motion_model_.w + sampling(variance2, this->params_.normalTriangularDistribution);
    double th_hilf = sampling(variance3, this->params_.normalTriangularDistribution);

    if(w_hat == 0.0 || v_hat == 0.0 || th_hilf == 0.0){
        return sample_old;
    }

    // kinematic model
    double x_ = sample_old.x - ((v_hat/w_hat) * sin(sample_old.th)) + ((v_hat/w_hat) * sin(sample_old.th + w_hat * this->dt_));
    double y_ = sample_old.y + ((v_hat/w_hat) * cos(sample_old.th)) - ((v_hat/w_hat) * cos(sample_old.th + w_hat * this->dt_));
    double th_ = sample_old.th + w_hat * this->dt_ + th_hilf * this->dt_;

    // speichern in Sample-struct und return
    Sample s1;
    s1.x = x_;
    s1.y = y_;
    s1.th = th_;
    return s1;
}

// Motion Model für ein sample mit Odometry
//nach Pseudocode Thrun S.110
Sample Filter::sample_motion_model_odometry(Sample sample_old){

    // δ == delta

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){

        double delta_rot_1 = atan2((this->odom_.y - this->odom_.y_d1), (this->odom_.x - this->odom_.x_d1)) - this->odom_.th_d1;
        double delta_trans = sqrt(pow((this->odom_.x_d1 - this->odom_.x), 2) + pow((this->odom_.y_d1 - this->odom_.y), 2));
        double delta_rot_2 = this->odom_.th - this->odom_.th_d1 - delta_rot_1;

        double variance1 = this->params_.alpha_odom_1 * delta_rot_1 + this->params_.alpha_odom_2 * delta_trans;
        double variance2 = this->params_.alpha_odom_3 * delta_trans + this->params_.alpha_odom_4 * (delta_rot_1 + delta_rot_2);
        double variance3 = this->params_.alpha_odom_1 * delta_rot_2 + this->params_.alpha_odom_2 * delta_trans;

        double delta_rot_1_hat = delta_rot_1 - sampling(variance1, this->params_.normalTriangularDistribution);
        double delta_trans_hat = delta_trans - sampling(variance2, this->params_.normalTriangularDistribution);
        double delta_rot_2_hat = delta_rot_2 - sampling(variance3, this->params_.normalTriangularDistribution);

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

// ========================================== Measurement Models ==========================================

// Measurement Model mit 2D laser scanner
// nach Thrun S.143
// Auswählbar mit sensorModelRangeFinder => true
double Filter::likelihood_field_range_finder_model(Sample sample)
{
    // wenn keine bewegung alles particles
    if(this->motion_model_.v < 0.1 && this->motion_model_.v > -0.1 && this->motion_model_.w < 0.1 && this->motion_model_.w > -0.1){
        // wenn keine bewegung --> alle particle gleich gewichten
        return (double)(1.0/(double)this->params_.anzSamples);
    }

    double probability = 1.0;

    for(int i = 0; i < this->sensor_model_.laserScan.size(); i++){ // für alle beams (i = angle of beam)
        if(this->sensor_model_.laserScan[i] >= this->sensor_model_.laserMinRange && this->sensor_model_.laserScan[i] < this->sensor_model_.laserMaxRange && i % this->params_.jederWieVielteBeam == 0){ // check if beam is valid
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
            double x_ztk = sample.x + this->params_.laserScannerPositionX * cos(sample.th) - this->params_.laserScannerPositionY * sin(sample.th) + this->sensor_model_.laserScan[i] * cos(sample.th + angleInRad);
            double y_ztk = sample.y + this->params_.laserScannerPositionY * cos(sample.th) + this->params_.laserScannerPositionX * sin(sample.th) + this->sensor_model_.laserScan[i] * sin(sample.th + angleInRad);

            // Zeile 7 --> dist²
            double minDist_2 = this->sensor_model_.laserMaxRange;

            for(int j = 0; j < this->map_.ray_castingPoints.size(); j++){
                // berechnen der euklidischen Distanz zu jedem validen Punkt (!= -1) --> dist²
                double dist_berechnet = pow((x_ztk - this->map_.ray_castingPoints[j].x), 2) + pow((y_ztk - this->map_.ray_castingPoints[j].y), 2);
                // übernehmen des Minimums
                if(dist_berechnet < minDist_2){
                    minDist_2 = dist_berechnet;
                }
            }

            // Zeile 8 --> berechnen von q (probability)
            probability = probability * (this->params_.zHit * probabilityDist(minDist_2, this->params_.sigmaHit) + (this->params_.zRand / this->params_.zMax));
        }
    }
    return probability;
}

// ---------- odometry measurement models ----------

// approach #1 mit linearer regelung von Siegward 2004
// genauere beschreibung siehe Paper
// ACHTUNG: funktioniert nur wenn alle Samples im robot origin [0.5, 0.5, 0] starten !!! (deshalb nicht auswählbar von .yaml)
double Filter::odom_vel_sensor_model_linearer_regler(Sample predictedSample, Sample oldSample){

    double gewichtung = 0.0;

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){
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

        // berechnnen der vels
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
        gewichtung = (double)(1.0/(double)this->params_.anzSamples);
    }
    
    return gewichtung;
}

// approach #2 mit Odometry
// genauere beschreibung siehe Paper
// ACHTUNG: funktioniert nur wenn alle Samples im robot origin [0.5, 0.5, 0] starten !!! (deshalb nicht auswählbar von .yaml)
double Filter::odom_vel_sensor_model(Sample predictedSample, Sample oldSample){
    double gewichtung = 0.0;

    if(this->motion_model_.v > 0.1 || this->motion_model_.v < -0.1 || this->motion_model_.w > 0.1 || this->motion_model_.w < -0.1){

        double v_odom = this->odom_.x_vel;
        double w_odom = this->odom_.th_vel;

        // berechnen von x, y, th aus den odom v und w
        double x_berechnet = oldSample.x - ((v_odom/w_odom) * sin(oldSample.th)) + ((v_odom/w_odom) * sin(oldSample.th + w_odom * this->dt_));
        double y_berechnet = oldSample.y + ((v_odom/w_odom) * cos(oldSample.th)) - ((v_odom/w_odom) * cos(oldSample.th + w_odom * this->dt_));
        double th_berechnet = oldSample.th + w_odom * this->dt_;

        // berechnen der Differenz und gewichten
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
        gewichtung = (double)(1.0/(double)this->params_.anzSamples);
    }

    return gewichtung;
}

// approach #3 mit Odometry-Sample
// genauere beschreibung siehe Paper
// Auswählbar mit sensorModelRangeFinder => false
// funktion für das updaten des odom-samples
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

// motion model -> vergleicht die samples mit odom-sample
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
        gewichtung = (double)(1.0/(double)this->params_.anzSamples);
    }

    return gewichtung;
}

// ========================================== resampling ==========================================

// nach Thrun S.86
std::vector<Sample> Filter::low_variance_sampler(std::vector<Sample> predictedSamples) {
    // empty set of samples
    std::vector<Sample> correctedSamples(this->params_.anzSamples);

    // random value
    double r = random_value(0.0, (double)(1.0/(double)this->params_.anzSamples));

    // Gewichte Normalisieren
    std::vector<double> normWeights(this->params_.anzSamples);
    double sumWeights = 0.0;
    for(int i = 0; i < this->params_.anzSamples; i++){
        sumWeights += predictedSamples[i].weight;
    }
    if(sumWeights != 0.0){
        for(int i = 0; i < this->params_.anzSamples; i++){
            normWeights[i] = predictedSamples[i].weight / sumWeights;
        }
    }

    double c = normWeights[0]; // cumulative sum
    int i = 0;                 // index for predictedSamples --> drawing
    int j = 0;                 // index for correctedSamples --> placing/speichern

    // draw with replacement
    for(int m = 0; m < this->params_.anzSamples; m++){
        double u = r + ((double) m * (double)(1.0/(double)this->params_.anzSamples));
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

// eigentlicher Algorithmus welcher models und resampling aufruft
// nach Thrun S.200
void Filter::algorithmMCL(){
    std::vector<Sample> samples_predicted(this->params_.anzSamples); // for sampling and weighting
    std::vector<Sample> samples_new(this->params_.anzSamples);      // for resampling

    updateOdomSample(); // Odom approach #3
    for(int i = 0; i < this->params_.anzSamples; i++){
        if(this->params_.motionModelOdom){
            samples_predicted[i] = sample_motion_model_odometry(this->samples_old_[i]);              // Motion Model mit Odometry
        }
        else
        {
            samples_predicted[i] = sample_motion_model_motion_command(this->samples_old_[i]);        // Motion Model mit Motion command
        }
        if(this->params_.sensorModelRangeFinder){
            samples_predicted[i].weight = likelihood_field_range_finder_model(samples_predicted[i]); // Measurement Model mit laser scanner
        }
        else
        {
            samples_predicted[i].weight = odom_vel_sensor_model_odomSample(samples_predicted[i]);                               // Odom approach #3
            //samples_predicted[i].weight = odom_vel_sensor_model(samples_predicted[i], this->samples_old_[i]);                 // Odom appraoch #2
            //samples_predicted[i].weight = odom_vel_sensor_model_linearer_regler(samples_predicted[i], this->samples_old_[i]); // Odom appraoch #1
        }
    }

    // Resampling
    samples_new = low_variance_sampler(samples_predicted);

    // Array übertragen an samples_old_
    for(int i = 0; i < this->params_.anzSamples; i++){
        if(this->params_.nurMotionModel){
            this->samples_old_[i] = samples_predicted[i]; // nur motion model
        }
        else
        {
            this->samples_old_[i] = samples_new[i]; // resampling wird übernommen
        }
    }
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

    // Ursprung [0, 0] --> liegt im Mittelpunkt der Map --> [160, 160] = [this->map_.height / 2, this->map_.width / 2] = [8m / 0.05res, 8m / 0.05res]
    // (8m = x-Richtung, 8m = -x-Richtung, 8m = y-Richtung, 8m = -y-Richtung)
    double ursprung_x = (this->map_.width * this->map_.resolution / 2);
    double ursprung_y = (this->map_.height * this->map_.resolution / 2);

    // kopieren des occupancy grids in mein 2D Array
    for(int zeile = 0; zeile < this->map_.height; zeile++){
        for(int spalte = 0; spalte < this->map_.width; spalte++){
            int index = zeile * this->map_.height + spalte;

            // befüllen des rayray_castingPoints für alle Zellenmittelpunkte wo nicht unbekannt
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
        }
    }
}

// ========================================== main ==========================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    // getting hyperparameters
    Hyperparameter param;

    // einlesen der parameter
    n.getParam("anzSamples", param.anzSamples);
    n.getParam("nurMotionModel", param.nurMotionModel);
    n.getParam("motionModelOdom", param.motionModelOdom);
    n.getParam("normalTriangularDistribution", param.normalTriangularDistribution);
    n.getParam("initialPosesNormalverteilt", param.initialPosesNormalverteilt);
    n.getParam("sensorModelRangeFinder", param.sensorModelRangeFinder);
    n.getParam("clearingPoses", param.clearingPoses);
    n.getParam("initialX", param.initialX);
    n.getParam("initialY", param.initialY);
    n.getParam("initialTH", param.initialTH);
    n.getParam("alpha_1", param.alpha_1);
    n.getParam("alpha_2", param.alpha_2);
    n.getParam("alpha_3", param.alpha_3);
    n.getParam("alpha_4", param.alpha_4);
    n.getParam("alpha_5", param.alpha_5);
    n.getParam("alpha_6", param.alpha_6);
    n.getParam("alpha_odom_1", param.alpha_odom_1);
    n.getParam("alpha_odom_2", param.alpha_odom_2);
    n.getParam("alpha_odom_3", param.alpha_odom_3);
    n.getParam("alpha_odom_4", param.alpha_odom_4);
    n.getParam("laserScannerPositionX", param.laserScannerPositionX);
    n.getParam("laserScannerPositionY", param.laserScannerPositionY);
    n.getParam("jederWieVielteBeam", param.jederWieVielteBeam);
    n.getParam("zHit", param.zHit);
    n.getParam("zShort", param.zShort);
    n.getParam("zMax", param.zMax);
    n.getParam("zRand", param.zRand);
    n.getParam("sigmaHit", param.sigmaHit);

    // creating filter object with parameters
    Filter filter(param);

    // subscribers
    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, &Filter::callback_cmd_vel, &filter);
    //ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom, &filter);        // speichern nur aktuelle Odometrie
    ros::Subscriber sub_odom = n.subscribe("odom", 1000, &Filter::callback_odom_motion, &filter);   // speichert Odometrie vom aktuellen und letztem time step
    ros::Subscriber sub_laser = n.subscribe("scan", 1000, &Filter::callback_laser, &filter);
    ros::Subscriber sub_occupancy_grid = n.subscribe("map", 1000, &Filter::callback_grid, &filter);

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
        if(filter.getHyperparameter().clearingPoses){
            sample_poses.poses.clear();
        }

        // MCL Angorithm
        filter.algorithmMCL();

        // vom filter umwandeln in eine pose + eintragen in pose_array
        for(int i = 0; i < filter.getHyperparameter().anzSamples; i++){
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
        std::cout << "Map width: " << filter.getMap().width << std::endl;
        std::cout << "Map height: " << filter.getMap().height << std::endl;

        std::cout << "---" << std::endl;
        std::cout << std::endl;

        //publishing sample poses
        pub_particle_cloud.publish(sample_poses);

    	loop_rate.sleep();
   	}

    return 0;
}