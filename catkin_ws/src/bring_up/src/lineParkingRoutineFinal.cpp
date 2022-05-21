#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64.h>//to publ
#include <std_msgs/Float32MultiArray.h>



class lineParkingClass
{
public:
     lineParkingClass(ros::NodeHandle nh) : n_(nh), priv_nh_("~")
     {
          subObj = n_.subscribe("/obstacles", 10000, &lineParkingClass::callback, this);
          speed_pub_=nh.advertise<std_msgs::Float64>("/speed",1);
          steering_pub_=nh.advertise<std_msgs::Float64>("/steering",1);
     }
     ~lineParkingClass(){}
     void callback(const std_msgs::Float32MultiArray& msgs);

private:
     ros::NodeHandle n_;
     ros::NodeHandle priv_nh_;
     ros::Subscriber subObj;
     ros::Publisher speed_pub_;
     ros::Publisher steering_pub_;
     double index; //from /obstacles array index 
     int step=0;//---------0
     int count=0;
     int auto_detect=0;
     int count_auto=0;
     float velocidad = 3.0;
};


void lineParkingClass::callback(const std_msgs::Float32MultiArray& msg)
{
     std_msgs::Float64 msgSpeed;
     std_msgs::Float64 msgSteering;
     index = msg.data[0];
     //ROS_INFO_STREAM("Auto: "<<index<<" count: "<<count_auto);

     //FORWARD
     //***************************************
     //step=1;                                   //DELETE
     //***************************************
     if (step==0)
     {
          if (index==1 && auto_detect==0)
          {
               count_auto+=1;
               auto_detect=1;
                    ROS_INFO_STREAM("Auto: "<<index<<" count: "<<count_auto);

          }
          else if (index==0 && auto_detect==1)
          {
               auto_detect=0;
          }
          
          if (count_auto<=1)
          {
               msgSpeed.data=velocidad +2;
               msgSteering.data=0.0;     
          } 
          else if (count_auto==2)
          {
               msgSpeed.data=velocidad;
               msgSteering.data=0.0;               
          }
          else
          {
               msgSpeed.data=0.0;
               msgSteering.data=0.0;
               step=1;//1            
          }
          speed_pub_.publish(msgSpeed);
          steering_pub_.publish(msgSteering);
     }
     //FORWARD AND TURN TO RIGHT
     else if(step==1)
     {
          ROS_INFO_STREAM("i: "<<step);                    
          if (count<9)
          { 
          //timeRoutine();
               msgSpeed.data=-velocidad;
               msgSteering.data=0.0;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               count+=1;
          }
          else if (count>=9 && count<12)
          {    
               msgSpeed.data=0.0;
               msgSteering.data=0.0;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               count+=1;
          }          
          else if (count>=12 && count<41)
          {    
               msgSpeed.data=-velocidad;
               msgSteering.data=0.52;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               count+=1;
          }
          else if (count>=41 && count<61)
          {    
               msgSpeed.data=-velocidad;
               msgSteering.data=-0.78;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               count+=1;
          }
          else if (count>=61 && count<67)
          {    
               msgSpeed.data=velocidad;
               msgSteering.data=-0.0;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               count+=1;
          }
          else
          {
               msgSpeed.data=0.0;
               msgSteering.data=-0.0;
               speed_pub_.publish(msgSpeed);
               steering_pub_.publish(msgSteering);
               step=2;
          }
     }
     else
     {
          ROS_INFO("END...");
     }
     
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "lineParkingRoutineFinal_node");
     ros::NodeHandle nh;
     lineParkingClass lineParking_instance(nh);
     ROS_INFO("RUN... lineParkingRoutineV1_node");
     
     while(ros::ok())
     {
          ros::spin();
     }
}
