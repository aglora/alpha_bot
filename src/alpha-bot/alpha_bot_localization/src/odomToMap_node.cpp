/* NODO QUE TRASLADA LOS TOPICS DE ODOM Y GROUND-TRUTH AL MARCO DE REFERENCIA MAP, PARA COMPARAR TODAS LAS FUENTES DE LOCALIZACIÓN DESDE EL MISMO FRAME

   Grupo Trabajo 14 CPR */


/* CABECERAS */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>                                      //Para convertir los cuaternios en rpy (y quedarnos solamente con yaw)
#include <nav_msgs/Odometry.h>                          //Tipo de dato para la pose en cada instante del robot
#include <nav_msgs/Path.h>                              //Tipo de dato de los waypoints
#include <geometry_msgs/Twist.h>                        //Tipo de dato de cmd_vel (velocidad lineal + velocidad angular)
#include <geometry_msgs/PoseStamped.h>                  //Tipo de dato dentro de las poses
#include <geometry_msgs/PoseWithCovarianceStamped.h>    //Tipo de dato al que queremos convertir para "robot_localization"
#include <cmath>                                        //Para el uso de las funciones "abs", "pow" y "fmod"
#include <iostream>                                     //Para el uso de malloc
#include <cstdlib>                                      //Para el uso de malloc
#include <vector>                                       //Para "size()" => cálculo del número de elementos de un vector


/* VARIABLES GLOBALES */
nav_msgs::Odometry poseActualGT;
nav_msgs::Odometry poseActualOdom;


/* FUNCIÓN DE LECTURA DE POSES: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de poses del topic de Ground Truth
   
*/

void posesGTCallback(const nav_msgs::Odometry& msg) {  
    poseActualGT = msg;
}

/* FUNCIÓN DE LECTURA DE POSES: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de poses del topic de /odom
   
*/

void posesOdomCallback(const nav_msgs::Odometry& msg) {  
    poseActualOdom = msg;
}


/* MAIN */

int main(int argc, char** argv){
  ros::init(argc, argv, "odomToMap_publisher");

  //PUBLICACIONES Y SUSCRIPCIONES
  ros::NodeHandle n;
  ros::Subscriber poseGT_sub = n.subscribe("/ground_truth_final", 50, posesGTCallback);                                /* Nombre del topic donde vamos a suscribirnos para obtener la pose actual del robot */
  ros::Subscriber poseOdom_sub = n.subscribe("/alpha_bot/mobile_base_controller/odom", 50, posesOdomCallback);         /* Nombre del topic donde vamos a suscribirnos para obtener la pose actual del robot */
  ros::Publisher poseGT_pub  = n.advertise<nav_msgs::Odometry>("/ground_truth_final_MAP", 50);                         /* Nombre del topic donde vamos a publicar */
  ros::Publisher poseOdom_pub  = n.advertise<nav_msgs::Odometry>("/alpha_bot/mobile_base_controller/odom_MAP", 50);    /* Nombre del topic donde vamos a publicar */

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Time contadorTiempo = ros::Time::now();
  /* BUCLE INFINITO */
  ros::Rate r(50.0);
  while(n.ok()){   

    ros::spinOnce();               
    current_time = ros::Time::now();

    poseActualGT.header.frame_id = "map";
    poseActualOdom.header.frame_id = "map";

    //PUBLICACIONES
    poseGT_pub.publish(poseActualGT);
    poseOdom_pub.publish(poseActualOdom);

    last_time = current_time;
    r.sleep();
  }
}

