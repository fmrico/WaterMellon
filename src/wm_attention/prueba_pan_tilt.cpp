/*
 * 
 * 
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 11/01/2015
 *  
 * Programa de prueba del cuello robótico FLIR
 * 
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/Led.h>
#include <geometry_msgs/Twist.h> // for velocity commands
#include <geometry_msgs/TwistStamped.h> // for velocity commands
#include <sstream>
#include "std_msgs/String.h"

int
main(int argc, char** argv)
{

  ros::init(argc, argv, "prueba_pan_tilt");		//Inicializa el nodo
  ros::NodeHandle n;

  ros::Publisher flir_pub;		//Declaramos los publicadores y subcriptores
  ros::Subscriber flir_sub;

  flir_pub 	= n.advertise<sensor_msgs::JointState>("/joint_states", 1); 
  sensor_msgs::JointState msg;
  msg.position.resize(2);
  msg.name.resize(2);
  msg.velocity.resize(2);
  msg.name[0] = "ptu_pan";
  msg.name[1] = "ptu_tilt";
  ros::Rate loop_rate(5);  
  float posPan = 0.0;   // Pan es el giro [-2,7 , 2,7]
  float posTilt = 0.0;  // Tilt la inclinacion [-0,82 , 0.52]
  bool retroceso = false;



  while (ros::ok())  //bucle principal, aqui entramos y nos mantenemos durante la ej
  {
    msg.header.stamp = ros::Time::now();
    msg.position[0] = posPan;
    msg.position[1] = posTilt;
    msg.velocity[0] = 0.7;
    msg.velocity[1] = 0.15;
    flir_pub.publish(msg);
    if ((posPan < 2.7 ) and (retroceso == false)){
       posPan = posPan + 0.2;
       posTilt = posTilt + 0.05;
    } 
    if (posPan > 2.7){
      retroceso = true;
    }
    if ((retroceso == true) and (posPan > (-2.7))){
      posPan = posPan - 0.2;
      posTilt = posTilt - 0.05;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}
