#include <ros/ros.h>                  /* CÓDIGO PARA TRASLADAR EL GROUND TRUTH VISUALMENTE DESDE LA POSICIÓN ARBITRARIA EN LA QUE EMPÌEZA HASTA LA POSICIÓN INICIAL DEL ROBOT */
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>

/* VARIABLES GLOBALES */
ros::Time current_time;

std::string frameID, child_frameID;   /* Tipo string del namespace std de C++ */
int headerSeq;
ros::Time headerStamp;

float pose_x, pose_y;
float vx,vy,vth;
nav_msgs::Odometry covarianzas;       /* Como no encontramos ningún tipo compatible con las covarianzas, usaremos un mensaje odometry entero para guardar 
                                         solo las covarianzas, en los campos covarianzas de pose y de velocidad (twist) */ 

geometry_msgs::Quaternion groundtruth_quat;

void groundtruthCallback(const nav_msgs::Odometry& msg) {     /*Cada vez que se llame a esta función, actualizar los siguientes parámetros */
  /*nav_msgs::Odometry groundtruth;
  geometry_msgs::TransformStamped groundtruth_trans;
  groundtruth.header.stamp = current_time;
  groundtruth_trans.header.stamp = current_time;*/


  // Actualizar pose_x, pose_y y groundtruth_quat: se toman del mensaje 
  //Actualizar todos los campos del topic al que me he suscrito:
  /*CAMPOS DE UN nav_msgs/Odomettry EXTRAIDOS DE UN rostopic echo */
  /*
  header: 
    seq: 1772
    stamp: 
      secs: 35
      nsecs: 598000000
    frame_id: "map"
  child_frame_id: "base_footprint"
  pose: 
    pose: 
      position: 
        x: -2.00022103511
        y: -0.500006260694
        z: -0.0500336306201
      orientation: 
        x: -1.21653992146e-06
        y: 0.00179337017611
        z: 0.000557919402944
        w: 0.999998236272
    covariance: [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
  twist: 
    twist: 
      linear: 
        x: 0.010007699104
        y: -0.0152720807663
        z: -0.00562976162021
      angular: 
        x: 0.0107310015785
        y: 0.0342468759728
        z: -0.0143605754152
    covariance: [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
    */

  /* Por su parte, la documentación de ROS especifica la composición del tipo de dato publicado en estos topics, el "nav_msgs/Odometry" como: */
  /*

   http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html

  # This represents an estimate of a position and velocity in free space.  
  # The pose in this message should be specified in the coordinate frame given by header.frame_id.
  # The twist in this message should be specified in the coordinate frame given by the child_frame_id
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist */


  frameID                      = msg.header.frame_id;  
  child_frameID                = msg.child_frame_id;
  headerSeq                    = msg.header.seq;
  headerStamp                  = msg.header.stamp;
  
  pose_x                       = msg.pose.pose.position.x;
  pose_y                       = msg.pose.pose.position.y;
  groundtruth_quat             = msg.pose.pose.orientation;
  covarianzas.pose.covariance  = msg.pose.covariance;


  vx                           = msg.twist.twist.linear.x;
  vy                           = msg.twist.twist.linear.y;
  vth                          = msg.twist.twist.angular.z;
  covarianzas.twist.covariance = msg.twist.covariance;

}


int main(int argc, char** argv){

  nav_msgs::Odometry groundtruth_final;
  geometry_msgs::TransformStamped groundtruth_final_trans;
  
  ros::init(argc, argv, "groundtruth_publisher");
  ros::NodeHandle n;

  ros::Subscriber groundtruth_sub = n.subscribe("ground_truth/state", 50, groundtruthCallback);  /* Nombre del topic donde vamos a suscribirnos */
  ros::Publisher groundtruth_pub  = n.advertise<nav_msgs::Odometry>("ground_truth_final", 50);   /* Nombre del topic donde vamos a publicar */

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while(ros::ok() && groundtruth_sub.getNumPublishers() == 0) {
    ros::Duration(0.2).sleep();
    ROS_INFO_THROTTLE(1.0, "Waiting for publishers...");
  }

  /* BUCLE INFINITO */
  ros::Rate r(50.0);  //Tasa de envío 
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //Llamar a la función para recoger el valor actual del ground_truth, descomponer el mensaje en componentes del /ground_truth/state

    /* Obtener transformación entre ground_truth/state y ground_truth_final  =====> es una traslación pura, por tanto, xGT,fin = xGT,ini + incTF */
    //Recomponer groundtruthfinal y enviarlo por /ground_truth_final
    groundtruth_final.header.frame_id       = "odom";               /* Este no lo copiamos del topic a la entrada, sino que lo remapeamos a odom */
    groundtruth_final.child_frame_id        = child_frameID;
    groundtruth_final.header.seq            = headerSeq;                    
    groundtruth_final.header.stamp          = headerStamp;              

    groundtruth_final.pose.pose.position.x  = pose_x+2;
    groundtruth_final.pose.pose.position.y  = pose_y+0.5;
    groundtruth_final.pose.pose.position.z  = 0.0;
    groundtruth_final.pose.pose.orientation = groundtruth_quat;
    groundtruth_final.pose.covariance       = covarianzas.pose.covariance;

    groundtruth_final.twist.twist.linear.x  = vx;
    groundtruth_final.twist.twist.linear.y  = vy; 
    groundtruth_final.twist.twist.angular.z = vth;                        
    groundtruth_final.twist.covariance      = covarianzas.twist.covariance;

    //Publicar el mensaje recompuesto
    groundtruth_pub.publish(groundtruth_final);

    last_time = current_time;
    r.sleep();
  }
}



