#include <ros/ros.h>
#include <manfred_arm_msgs/Write_pmac.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <string.h>
//#include "pmac.h"

using namespace std;

class PmacWriter
{
	public:
		PmacWriter();
	private:
        string convert (const float number);
		void jsCallback (const sensor_msgs::JointState::ConstPtr& vel);
		ros::ServiceClient client;
		ros::Subscriber sub;
        ros::Publisher pub;
		ros::NodeHandle ph, nh;
		manfred_arm_msgs::Write_pmac srv;
		string velocidad_pordefecto1;
		string velocidad_pordefecto2;
		string movimiento;
};

PmacWriter::PmacWriter()
{
	
	client = nh.serviceClient<manfred_arm_msgs::Write_pmac>("pmac_writer_service");
	sub = ph.subscribe<sensor_msgs::JointState>("wheel_movement", 10, &PmacWriter::jsCallback,this);
    pub = ph.advertise<sensor_msgs::JointState>("wheel_velocities",1);
	
}

string PmacWriter::convert(const float number)
{
	ostringstream buff;
    buff<<abs(number);
    return buff.str();	
}

void PmacWriter::jsCallback(const sensor_msgs::JointState::ConstPtr& vel)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.velocity.resize(2);
    joint_state.position.resize(2);
    float wRight= vel->velocity[0];
    float wLeft= vel->velocity[1];
    joint_state.velocity[0]=wRight;
    joint_state.velocity[1]=wLeft;
    string wRight_ = PmacWriter::convert(wRight);
    string wLeft_ = PmacWriter::convert(wLeft);
    velocidad_pordefecto1 = "I122=" + wRight_;
    cout << "\nComando PMAC:" << velocidad_pordefecto1 << endl;
    velocidad_pordefecto2 = "I222=" + wLeft_;
    cout << "\nComando PMAC:" << velocidad_pordefecto2 << endl;
    if (wRight>0 && wLeft>0){movimiento = "#1J+#2J+";}
    else if (wRight>0 && wLeft<0){movimiento = "#1J+#2J-";}
		else if (wRight<0 && wLeft>0){movimiento = "#1J-#2J+";}
			else if (wRight<0 && wLeft<0){movimiento = "#1J-#2J-";}
				else if (wRight==0 && wLeft==0){movimiento = "#1J^0#2J^0";}   
	cout << "\nComando PMAC:" << movimiento << endl; 
	   
	srv.request.send = velocidad_pordefecto1;
	if (client.call(srv)){
	    cout << "Servidor responde: " << srv.response.answer << endl;
	}
	else{
	    ROS_INFO("Failed to call service");
	}
    
    srv.request.send = velocidad_pordefecto2;
	if (client.call(srv)){
	    cout << "Servidor responde: " << srv.response.answer << endl;
	}
	else{
	    ROS_INFO("Failed to call service");
	}
	
    srv.request.send = movimiento;
	if (client.call(srv)){
	    cout << "Servidor responde: " << srv.response.answer << endl;
	}
	else{
	    ROS_INFO("Failed to call service");
	}
     pub.publish(joint_state);
    } 


int main(int argc, char **argv){
  ros::init(argc, argv, "node_serv_client");
  PmacWriter pmac_writer; 
 
  ros::spin();
}
