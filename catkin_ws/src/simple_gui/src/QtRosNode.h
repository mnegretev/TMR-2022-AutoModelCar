#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();
    
    float current_speed;    //Speed is intended to be given in m/s
    float current_steering; //Steering is intended to be given in rad
    
    ros::NodeHandle* n;
    ros::Publisher pub_speed;
    ros::Publisher pub_steering;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_speed(double speed);
    void publish_steering(double steering);
    void update_speed_and_publish(double delta_speed);
    void update_steering_and_publish(double delta_steering);
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
