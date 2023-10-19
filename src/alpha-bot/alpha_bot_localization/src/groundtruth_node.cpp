/* NODO QUE TOMA LA POSE DEL TOPIC DE GROUND TRUTH "/ground_truth_final" PARA COPIARLA EN "/odometry/filtered", DONDE SE SUSCRIBEN LOS NODOS
   QUE DESEAN CONOCER LA POSE DEL ROBOT

   ADEMÁS, PUBLICA LA TRANSFORMACIÓN ENTRE MARCOS DE REFERENCIA DESDE ODOM A BASE_LINK, ADEMÁS DE ODOM A MAP (COINCIDEN)
   ENLACES DE INTERÉS:
   http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
   https://stackoverflow.com/questions/41156626/quaternion-0-0-0-0-signifies-what

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
nav_msgs::Odometry poseActual;


/* FUNCIÓN DE LECTURA DE POSES: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de poses del topic de Ground Truth
   
*/

void posesCallback(const nav_msgs::Odometry& msg) {  
    poseActual = msg;
}


/* MAIN */

int main(int argc, char** argv){
  ros::init(argc, argv, "laserOdometry_publisher");

  //PUBLICACIONES Y SUSCRIPCIONES
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/ground_truth_final", 50, posesCallback);           /* Nombre del topic donde vamos a suscribirnos para obtener la pose actual del robot */
  ros::Publisher pose_pub  = n.advertise<nav_msgs::Odometry>("/odometry/filtered", 50);       /* Nombre del topic donde vamos a publicar */

  tf::TransformBroadcaster odom_broadcaster;
  tf::Transform odomToMap;
  tf::Transform odomToBaseLink;
  tf::TransformListener tf_listener;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Time contadorTiempo = ros::Time::now();
  /* BUCLE INFINITO */
  ros::Rate r(50.0);
  while(n.ok()){   

    ros::spinOnce();               
    current_time = ros::Time::now();

    if((current_time - contadorTiempo) > (ros::Duration)0.6){
    //PUBLICAR TRANSFORMACIÓN DE ODOM A MAP
    odomToMap.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion qOdomToMap = {0.0, 0.0, 0.0, 1.0};
    odomToMap.setRotation(qOdomToMap);
    odom_broadcaster.sendTransform(tf::StampedTransform(odomToMap, ros::Time::now(), "odom", "map"));

    //PUBLICAR TRANSFORMACIÓN DE ODOM A BASE_LINK
    odomToBaseLink.setOrigin( tf::Vector3(poseActual.pose.pose.position.x, poseActual.pose.pose.position.y, poseActual.pose.pose.position.z));
    tf::Quaternion qOdomToBaseLink;
    qOdomToBaseLink = {poseActual.pose.pose.orientation.x, poseActual.pose.pose.orientation.y, poseActual.pose.pose.orientation.z, poseActual.pose.pose.orientation.w};
    if(abs(qOdomToBaseLink.getAngle())==nan("")) qOdomToBaseLink = {0.0,0.0,0.0,1.0};
    odomToBaseLink.setRotation(qOdomToBaseLink);
    tf_listener.waitForTransform("/map","/odom", ros::Time(), ros::Duration(1.0));
    odom_broadcaster.sendTransform(tf::StampedTransform(odomToBaseLink, ros::Time::now(), "odom", "base_link"));
    }


    //PUBLICACIONES EN TOPICS
    pose_pub.publish(poseActual);

    last_time = current_time;
    r.sleep();
  }
}

