#include "ros/ros.h"
#include "rover1_motordriver/Velocity.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include <math.h>

class PositionHandler{
public:
    PositionHandler(geometry_msgs::Point32 initPos, float theta, float radius, float timeInterval){
        m_position = initPos;
        m_radius = radius;
        m_diameter = 0.285;
        m_timeInterval = timeInterval;
        m_theta = theta;
    }

    void updatePosition(rover1_motordriver::Velocity vel){

        float diffVel = (vel.right_vel - vel.left_vel);
        float sumVel = (vel.right_vel + vel.left_vel);
        float omega = (diffVel)/m_diameter;
        float oldTheta = m_theta;

        m_theta = m_theta + omega*m_timeInterval;
	    m_theta = fmod(m_theta, 2*3.14);
	    if(m_theta > 3.14) m_theta = m_theta-2*3.14;
	    if(m_theta < -3.14) m_theta = m_theta + 2*3.14;

        m_velocity = sumVel/2;

        if (omega != 0){
            m_position.x = m_position.x + (m_velocity/omega)*(sin(m_theta) - sin(oldTheta));
            m_position.y = m_position.y - (m_velocity/omega)*(cos(m_theta) - cos(oldTheta));
        }
        else{
            m_position.x = m_position.x + m_velocity*cos(oldTheta);
            m_position.y = m_position.y + m_velocity*sin(oldTheta);
        }

    }

    geometry_msgs::Point32 &getPosition(){
        return m_position;
    }

    std_msgs::Float32 getTheta(){
        std_msgs::Float32 temp;
        temp.data = m_theta;
        return temp;
    }
private:
    geometry_msgs::Point32 m_position;
    float m_theta = 0;
    float m_radius = 1;
    float m_diameter = 2;
    float m_velocity = 0;
    double m_timeInterval = 0;
};

int main(int argc, char **argv)
{
    if(argc < 4){
        ROS_INFO("Please start node with: rosrun rover1_motordriver odometry 'x' 'y' 'theta'\n");
        return 0;
    }
    geometry_msgs::Point32 initPos;

    initPos.x = atof(argv[1]);
    initPos.y = atof(argv[2]);
    float theta = atof(argv[3]);

    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    ros::Publisher odometryPublisher = nh.advertise<geometry_msgs::Point32>("odometry", 10);
    ros::Publisher thetaPublisher = nh.advertise<std_msgs::Float32>("theta", 10);
    ros::Rate loopRate(10);

    PositionHandler positionHandler(initPos, theta, 0.06, 0.2);

    ros::Subscriber velocitySubscriber = nh.subscribe("/est_vel", 1, &PositionHandler::updatePosition, &positionHandler);

    while(ros::ok()){
        odometryPublisher.publish(positionHandler.getPosition());
        thetaPublisher.publish(positionHandler.getTheta());
        ros::spinOnce();
        loopRate.sleep();
    }
}
