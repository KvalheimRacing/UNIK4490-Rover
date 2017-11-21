#include "ros/ros.h"
#include "rover1_motordriver/Velocity.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <iostream>

class PostureHandler{
public:
    PostureHandler(float k1, float k2, float k3, float diameter, float radius){
        m_angularVelocity.right_vel = 0;
        m_angularVelocity.left_vel = 0;
        m_k1 = k1;
        m_k2 = k2;
        m_k3 = k3;
        m_diameter = diameter;
        m_radius = radius;
        m_position.x = 0;
        m_position.y = 0;
    }


    void updatePosition(geometry_msgs::Point32 position){
        m_position = position;
    }


    void updateTheta(std_msgs::Float32 theta){
        m_theta = theta;
    }


    rover1_motordriver::Velocity calculateVelocities(){

        // Check if position is acceptable (less than 0.05 deviation)
        if(!(m_position.x < 0.05 && m_position.y < 0.05) || !(m_position.x > -0.05 && m_position.y > -0.05)){

            m_rho = sqrt(m_position.x*m_position.x + m_position.y*m_position.y);
            m_gamma = fmod((atan2(m_position.y, m_position.x) - m_theta.data + 3.14), (2*3.14));

            if (m_gamma > 3.14) {
                m_gamma = (m_gamma - 2*3.14);
            }
            else if (m_gamma < -3.14) {
                m_gamma = (m_gamma + 2*3.14);
            }

            m_delta = m_gamma + m_theta.data;

            // Calculate velocities
            m_angularVelocity.right_vel = (2*m_k1*m_rho*cos(m_gamma) + m_diameter*m_k2*m_gamma + (m_diameter*m_k1*sin(m_gamma)*cos(m_gamma)/m_gamma)*(m_gamma + m_k3*m_delta)) / (2*m_radius);

            m_angularVelocity.left_vel = (2*m_k1*m_rho*cos(m_gamma) - m_diameter*m_k2*m_gamma - m_k1*m_diameter*(sin(m_gamma)*cos(m_gamma)/m_gamma)*(m_gamma + m_k3*m_delta)) / (2*m_radius);
        }
        else { // Stop moving if position is acceptable (because of controller design limitation)
            m_angularVelocity.right_vel = 0;
            m_angularVelocity.left_vel = 0;
        }

        std::cout << "Angular Velocity: right: " << m_angularVelocity.right_vel << "\t left: " << m_angularVelocity.left_vel <<"\t vinkel:" <<m_theta << "\n\n";
        return m_angularVelocity;
    }


private:
    rover1_motordriver::Velocity m_angularVelocity;
    geometry_msgs::Point32 m_position;
    std_msgs::Float32 m_theta;
    float m_k1 = 1;
    float m_k2 = 1;
    float m_k3 = 1;
    float m_rho = 0;
    float m_gamma = 0;
    float m_delta = 0; // new
    float m_diameter = 1;
    float m_radius = 1;
};


int main(int argc, char **argv) {

    if(argc < 4){
        ROS_INFO("Please start node with: rosrun rover1_motordriver posture_controller 'k1' 'k2' 'k3'\n");
        return 0;
    }

    ros::init(argc, argv, "posture_controller_node");
    ros::NodeHandle nh;
    ros::Publisher angularVelocityPublisher = nh.advertise<rover1_motordriver::Velocity>("/cmd_vel", 10);

    PostureHandler postureHandler(atof(argv[1]), atof(argv[2]), atof(argv[3]), 0.335, 0.06);

    ros::Subscriber positionSubscriber = nh.subscribe("/odometry", 1, &PostureHandler::updatePosition, &postureHandler);
    ros::Subscriber thetaSubscriber = nh.subscribe("/theta", 1, &PostureHandler::updateTheta, &postureHandler);

    ros::Rate loopRate(5);

    while(ros::ok()){
        ros::spinOnce();
        angularVelocityPublisher.publish(postureHandler.calculateVelocities());
        loopRate.sleep();
    }
}
