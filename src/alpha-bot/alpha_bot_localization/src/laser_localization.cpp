/* NODO PARA TOMAR LAS ESTIMACIONES DEL NODO "laser_scan_match" Y CONVERTIRLO A MENSAJE DE TIPO "geometry_msgs::PoseWithCovarianceStamped" PARA PODER SER USADO
   EN EL EKF Y ASÍ MEJORAR SU COMPORTAMIENTO

   ENLACES DE INTERÉS
   https://answers.ros.org/question/331081/does-move_base-require-twist-from-odometry/
   https://answers.ros.org/question/57859/is-amcl-required-together-with-laser_scan_matcher-for-navigation/
   https://answers.ros.org/question/331044/is-it-possible-to-convert-geometry_msgs-posestamped-to-nav_msgs-odometry/
   http://wiki.ros.org/laser_scan_matcher

   Grupo Trabajo 14 CPR */


/* CABECERAS */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
geometry_msgs::PoseStamped poseActual;


/* FUNCIÓN DE LECTURA DE POSES: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de poses del topic de waypoints, en el que publica el planificador
   de caminos 
   
   Estructura de un mensaje tipo geometry_msgs/PoseStamped:
   # A Pose with reference coordinate frame and timestamp
   std_msgs/Header header
        uint32 seq                              # Sequence ID: consecutively increasing ID 
        time stamp                              # Two-integer timestamp
        string frame_id                         # Frame this data is associated with
   
   geometry_msgs/Pose pose
        geometry_msgs/Point position            # This contains the position of a point in free space
                float64 x                  
                float64 y
                float64 z
        geometry_msgs/Quaternion orientation    # This represents an orientation in free space in quaternion form.
                float64 x
                float64 y
                float64 z
                float64 w


   Enlaces de interés:
   https://stackoverflow.com/questions/72358077/cannot-show-the-published-path-in-rviz-ros
   https://www.programcreek.com/cpp/?CodeExample=geometry+msgs+pose+stamped
*/

void posesCallback(const geometry_msgs::PoseStamped& msg) {  
    poseActual = msg;
}


/* MAIN */

int main(int argc, char** argv){
  ros::init(argc, argv, "laserOdometry_publisher");

  //PUBLICACIONES Y SUSCRIPCIONES
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/pose_stamped", 50, posesCallback);                                       /* Nombre del topic donde vamos a suscribirnos para obtener la pose actual del robot */
  ros::Publisher pose_pub  = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laser_pose", 50);              /* Nombre del topic donde vamos a publicar */

  tf::TransformBroadcaster waypoint_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /* Variables locales de main */
  geometry_msgs::PoseWithCovarianceStamped odometriaLaser;  //Pose calculada a partir de las medidas del láser

  /* BUCLE INFINITO */
  ros::Rate r(50.0);
  while(n.ok()){

    ros::spinOnce();               
    current_time = ros::Time::now();
    
    //Reconstrucción del mensaje "geometry_msgs::PoseWithCovarianceStamped odometriaLaser final"
    odometriaLaser.header.frame_id = "map";

    //POSE
    //Pose.Position
    odometriaLaser.pose.pose.position.x = poseActual.pose.position.x;
    odometriaLaser.pose.pose.position.y = poseActual.pose.position.y;
    odometriaLaser.pose.pose.position.z = poseActual.pose.position.z;
    odometriaLaser.pose.pose.orientation.x = poseActual.pose.orientation.x;
    odometriaLaser.pose.pose.orientation.y = poseActual.pose.orientation.y;
    odometriaLaser.pose.pose.orientation.z = poseActual.pose.orientation.z;
    odometriaLaser.pose.pose.orientation.w = poseActual.pose.orientation.w;

    //Pose.Covariance
    
    odometriaLaser.pose.covariance[0]  = 0.0; //x
    odometriaLaser.pose.covariance[6]  = 0.0; //y
    odometriaLaser.pose.covariance[35] = 0.0; //th
    

    /*
    odometriaLaser.pose.covariance[0] = 0.05;
    odometriaLaser.pose.covariance[7] = 0.05;
    odometriaLaser.pose.covariance[14] = 0.05;
    odometriaLaser.pose.covariance[21] = 0.01;
    odometriaLaser.pose.covariance[28] = 0.01;
    odometriaLaser.pose.covariance[35] = 0.01;
    */

    //PUBLICACIONES
    pose_pub.publish(odometriaLaser);

    last_time = current_time;
    r.sleep();
  }
}

