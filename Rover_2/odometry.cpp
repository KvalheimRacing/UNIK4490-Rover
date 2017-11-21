#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "rover2_motordriver/Velocity.h"
#include "geometry_msgs/Point32.h"

class PositionByOdometry{
    public:
            PositionByOdometry(geometry_msgs::Point32 position, float angular_velocity_l, float angular_velocity_r, float theta, float wheel_radius, float delta_t, float traversial_wheelbase){
                this->position=position;                            // Desired position
                this->wheel_radius=wheel_radius;                    // Radius from center of the axle
                this->delta_t=delta_t;                              // Sampling time intervall
                this->theta=theta;                                  // Desired angle
                this->angular_velocity_l=angular_velocity_l;        // Rotational/angular velocity in rad/s for left wheels
                this->angular_velocity_r=angular_velocity_r;        // Rotational/angular velocity in rad/s for right wheels
                double pi = 3.141592653589793238462643383279502884; // Approx of Pi
            }

            // Calculating variables to know where the rover are in the world coordinate frame
            // The method used is odometry by exact integration, and can be found in chapter 11.7 in "Robotics - Modelling, Planning and Control" by B. Siciliano et.al (equations 11.85)
            void odometry_calculations(rover2_motordriver::Velocity vel){

                // Calculating velocities with regards to world coordinates
                velocity = ((Wr-Wl)/2)*1.19; // (wheel_radius*(Wr-Wl))/2; <- Old, taken away because of /est_vel is publishing m/s angular velocity. Multiplying with 1.19 to compensate for odometric fault

                // Calculating angular velocity with regards to world coordinates
                float ohmega = (Wr-Wl)/traversial_wheelbase; // (wheel_radius*(Wr-Wl))/traversial_wheelbase; <- Old, taken away because of /est_vel is publishing m/s angular velocity

                    // ohmega = 0 will only occur if something went wrong in driver_pid.py (or driver.py)
                    if (ohmega not 0) {

                        float newtheta = theta + ohmega*delta_t                                     // Calculating new angle relative to world coordinates
                        newtheta = fmod(newtheta, 2*pi);                                            // Returning the modulus of the angle (up to 2pi)
                	    if(newtheta > pi) newtheta = newtheta - 2*pi;                               // Limiting the angle
                	    if(newtheta < -pi) newtheta = newtheta + 2*pi;                              // Limiting the angle
                        position.x = position.x + (velocity/ohmega)*(sin(newtheta) - sin(theta));   // Calculating new x position relative to world coords
                        position.y = position.y - (velocity/ohmega)*(cos(newtheta) - cos(theta));   // Calculating new y position relative to world coords

                    }

            }

            // Gets position
            geometry_msgs::Point32 &getPosition(){
                return position;
            }

            // Gets theta
            std_msgs::Float32 getTheta(){
                std_msgs::Float32 tmp;
                tmp.data = theta;
                return tmp;
            }

            ~PositionByOdometry(); // Destructor

    private:
        geometry_msgs::Point32 position;            // Position message containing x and y position
        float angular_velocity_r, angular_velocity_l, theta, wheel_radius, velocity, delta_t, traversial_wheelbase;

};


int main(int argc, char **argv) {

    // Printing in terminal if missing inputs
    if(ar < 4) {
        ROS_INFO("Nah, that won't work, try: rosrun rover2_motordriver odomerty 'x' 'y 'theta'\n");
    }

    float wheel_radius = 0,006         // 6cm radius from center axle
    float wheel_circumfrence = 0,385;  // 38,5cm around the tire
    float traversial_wheelbase = 0,32  // distance between left and right side wheels
    int T_s = 0.1;                     // intervall between sampling in s (100ms) or (10Hz) - delta time between sampling of encoder positions

    // Setting startpositions
    geometry_msgs::Point32 position;
    position.x = atof(argv[1]);
    position.y = atof(argv[2]);
    float theta = atof(argv[3]);

    // Creating ros node, and publishers for odometry and angle
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    ros::Publisher odometryPublisher = nh.advertise<geometry_msgs::Point32>("odometry", 10);
    ros::Publisher thetaPublisher = nh.advertise<std_msgs::Float32>("theta", 10);
    ros::Rate loopRate(10);

    // Create an instance of PositionByOdometry
    PositionByOdometry positionByOdometry(position, theta, wheel_radius, T_s);

    // Subscribe to the topic /est_vel for inputs
    ros::Subscriber velocitySubscriber = nh.subscribe("/est_vel", 1, &positionByOdometry::odometry_calculations, &positionByOdometry);

    // Publish odometry and angle as long as a rosmaster/roscore runs
    while(ros::ok(){

        odometryPublisher.publish(positionByOdometry.getPosition());
        thetaPublisher.publish(positionByOdometry.getTheta());
        ros::spinOnce();
        loopRate.sleep();

    }

    return 0;

}
