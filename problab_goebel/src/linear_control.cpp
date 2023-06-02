#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


double goal [4][3] ={{4,0,0},{5,-2,-M_PI/2},{4,-3,-M_PI},{1,-1,M_PI}};


/*
odom für --> x, y, th
*/
double particle_set;

int goalreach = 0;
double tolz = 0.1; 
double x = 0;
double y = 0;
double th = 0;
double vt =0;
double wt =0;
//P-Regler Gewichtung
double kp = 0.4; 
double ka = 1;
double kb = -0.3; // nach Siegwart (2004)
   
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	th=tf::getYaw(odom->pose.pose.orientation);
	x=odom->pose.pose.position.x;
	y=odom->pose.pose.position.y;
}

void linearrechner(double x, double y, double th);

 int main(int argc, char **argv)
 {
   double lr=10;		//looprate
   double pX=0;
   double pY=0;
   double pT=0;
   
   ros::init(argc, argv, "publischer1");
   
   ros::NodeHandle n;	//node handle
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //publischer node erstellen //messagetyp topicname und buffer
   ros::Subscriber sub = n.subscribe("odom", 100,odom_callback);
   geometry_msgs::Twist posetwist;
  
   //Parameter einlesen
   n.getParam("x",pX);
   n.getParam("y",pY);
   n.getParam("t",pT);
   
   ros::Rate loop_rate(lr);	//Spinnen mit der rate 10Hz //////////////////////////////////////////////////////

   while(n.ok())
	{

		ros::spinOnce();               // check for incoming messages 
		
		//Parameter goals
		pX=goal[goalreach][0];
		pY=goal[goalreach][1];
		pT=goal[goalreach][2];

		linearrechner (pX,pY,pT);
			
		posetwist.linear.x=vt;
		posetwist.angular.z=wt;
		
		
		pub.publish(posetwist);
    	loop_rate.sleep();
   	}
 }
 
 
 void linearrechner(double pX, double pY, double pT)
 {
 	double dX = pX-x; 				
	double dY = pY-y;
	double dTh = pT-th;

	double p = sqrt((dX*dX)+(dY*dY));
	double alpha = -th + atan2(dY,dX);
	if(alpha > M_PI){alpha = alpha-2*M_PI;} // Schönes anfahren nicht übersteueren // Q:Vorlesung //
	if(alpha < -M_PI){alpha = alpha+2*M_PI;}
	double beta = dTh-alpha;
	if(beta > M_PI){beta = beta-2*M_PI;} // Schönes anfahren nicht übersteueren // Q:Vorlesung //
	if(beta < -M_PI){beta = beta+2*M_PI;}
	

	//Geschwindigkeit + RegelFaktor // Winkelgeschwindigkeit + Regelfaktor // Globale variablen => kein return
	vt = kp*p;
	wt = ka*alpha + kb*beta;
    
    	if(vt >= 0.21){vt= 0.2;}
    	if(vt <= -0.21){vt= -0.2;}
    	if(wt >= 2.3){wt= 2;}
    	if(wt <= -2.3){wt= -2;}
    	if(dX > -tolz && dX < tolz){if(dY > -tolz && dY < tolz){vt=0;/*if(dTh>=0){wt=1;}else{wt=-1;}*/if(dTh > -tolz*2 && dTh < tolz*2){wt=0;goalreach++;if(goalreach>3){goalreach=3;}}}}
    	
    	ROS_INFO_STREAM("|dth:" <<dTh <<"|th:"<<th <<"|Y:" <<dY <<"|alp:"<<alpha<<"|b:"<<beta <<"|p:"<<p <<"|v:"<< vt <<"|w:" << wt); // Wichtige Parameter Publischen =>/rosout
    
 }