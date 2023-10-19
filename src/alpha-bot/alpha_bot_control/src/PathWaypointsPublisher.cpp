/* NODO QUE CREA UN NUEVO TOPIC, "/PlannedPath", Y PUBLICA EN ÉL UN VECTOR DE WAYPOINTS A SEGUIR */
/* **** DE MOMENTO ESTÁTICOS: POSTERIORMENTE, QUE VAYA ROTANDO => EMULA LA EVOLUCIÓN DE LA REPLANIFICACIÓN */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>                        //Para convertir los cuaternios en rpy (y quedarnos solamente con yaw)
#include <nav_msgs/Odometry.h>            //Tipo de dato para la pose en cada instante del robot
#include <nav_msgs/Path.h>                //Tipo de dato de los waypoints
#include <geometry_msgs/Twist.h>          //Tipo de dato de cmd_vel (velocidad lineal + velocidad angular)
#include <geometry_msgs/PoseStamped.h>    //Tipo de dato dentro de los waypoints
#include <cmath>                          //Para el uso de la función "abs"
#include <iostream>                       //Para el uso de malloc
#include <cstdlib>                        //Para el uso de malloc

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoints_publisher");

  ros::NodeHandle n;
  ros::Publisher waypoints_pub  = n.advertise<nav_msgs::Path>("PlannedPath", 50); /* Nombre del topic donde vamos a publicar */
  
  /* LISTA DE WAYPOINTS (Carpeta "alpha_bot/alpha_bot_control/Waypoints") */
  //Se está ignorando la temporización: los instantes de tiempo en que se deben alcanzar los waypoints se añadirían en el header
  int i;
  nav_msgs::Path waypoints1;
  nav_msgs::Path waypoints2; 
  nav_msgs::Path waypoints3;   
  geometry_msgs::PoseStamped WPPoses[7];
  //Ha sido necesario retocar las posiciones, pues aunque en las fotos de la carpeta "Waypoints" el robot está en posiciones
  //lógicas, entre los waypoints 5 y 7 el robot intenta atravesar las columnas centrales, la trayectoria se ha deformado ligeramente (ver "Waypoints/CapturaRVIZCaminoTalCual.png")
  //Quizá deberíamos haber retocado también las orientaciones, pero no se ha hecho
  
  //double x[7] = {0.0365, 0.2092, 0.7141, 1.3999, 2.3483, 2.8359, 2.8597};
  //double y[7] = {-0.6158, -1.0200, -1.2761, -0.8949, -0.5071, 0.2403, 0.8366};
  double x1[7]  = {0.0365, 0.2092, 0.7141, 1.3999, 2.3483, 3.1359, 2.8597};
  double y1[7]  = {-0.6158, -1.0200, -1.2761, -0.8949, -0.8071, 0.2403, 0.7366};
  double qz1[7] = {-0.6169, -0.5446, -0.2215, 0.1817, 0.2037, 0.4890, 0.7010};
  double qw1[7] = {0.7871, 0.8387, 0.9752, 0.9834, 0.9790, 0.8723, 0.7132};
  
  double x2[3]  = {3.0289, 2.8181, 1.8957};   
  double y2[3]  = {1.2720, 1.7150, 1.9072};
  double qz2[3] = {-0.7121, -0.9670, -0.9852};
  double qw2[3] = {-0.7021, -0.2549, -0.1715};

  double x3[3]  = {1.2623, 0.5110, 0.3776};  
  double y3[3]  = {1.7782, 1.7578, 1.4509};
  double qz3[3] = {-0.9995, -0.9735, -0.8103};
  double qw3[3] = {0.0326, -0.2287, 0.5860};  

  for(i=0;i<7;i++){
    WPPoses[i].header.frame_id = "odom";
    WPPoses[i].pose.position.x = x1[i]; 
    WPPoses[i].pose.position.y = y1[i];
    WPPoses[i].pose.position.z = 0.0;
    WPPoses[i].pose.orientation.x = 0.0;
    WPPoses[i].pose.orientation.y = 0.0;
    WPPoses[i].pose.orientation.z = qz1[i];
    WPPoses[i].pose.orientation.w = qw1[i];
    waypoints1.poses.push_back(WPPoses[i]);               //https://answers.ros.org/question/282094/publish-a-path/
                                                         //https://stackoverflow.com/questions/72358077/cannot-show-the-published-path-in-rviz-ros
                                                         //https://www.programcreek.com/cpp/?CodeExample=geometry+msgs+pose+stamped
                                                         //CUIDADO: https://answers.ros.org/question/196840/nav_msgspath-without-the-field-pose/
  }
  waypoints1.header.frame_id="odom";

  for(i=0;i<3;i++){
    WPPoses[i].header.frame_id = "odom";
    WPPoses[i].pose.position.x = x2[i]; 
    WPPoses[i].pose.position.y = y2[i];
    WPPoses[i].pose.position.z = 0.0;
    WPPoses[i].pose.orientation.x = 0.0;
    WPPoses[i].pose.orientation.y = 0.0;
    WPPoses[i].pose.orientation.z = qz2[i];
    WPPoses[i].pose.orientation.w = qw2[i];
    waypoints2.poses.push_back(WPPoses[i]);              //https://answers.ros.org/question/282094/publish-a-path/
                                                         //https://stackoverflow.com/questions/72358077/cannot-show-the-published-path-in-rviz-ros
                                                         //https://www.programcreek.com/cpp/?CodeExample=geometry+msgs+pose+stamped
                                                         //CUIDADO: https://answers.ros.org/question/196840/nav_msgspath-without-the-field-pose/
  }
  waypoints2.header.frame_id="odom";

  for(i=0;i<3;i++){
    WPPoses[i].header.frame_id = "odom";
    WPPoses[i].pose.position.x = x3[i]; 
    WPPoses[i].pose.position.y = y3[i];
    WPPoses[i].pose.position.z = 0.0;
    WPPoses[i].pose.orientation.x = 0.0;
    WPPoses[i].pose.orientation.y = 0.0;
    WPPoses[i].pose.orientation.z = qz3[i];
    WPPoses[i].pose.orientation.w = qw3[i];
    waypoints3.poses.push_back(WPPoses[i]);              //https://answers.ros.org/question/282094/publish-a-path/
                                                         //https://stackoverflow.com/questions/72358077/cannot-show-the-published-path-in-rviz-ros
                                                         //https://www.programcreek.com/cpp/?CodeExample=geometry+msgs+pose+stamped
                                                         //CUIDADO: https://answers.ros.org/question/196840/nav_msgspath-without-the-field-pose/
  }
  waypoints3.header.frame_id="odom";

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /* BUCLE INFINITO */
  int bucle = 1;
  ros::Rate r(50.0);
  ros::Time contadorTiempo = ros::Time::now();
  while(n.ok()){

    ros::spinOnce();               
    current_time = ros::Time::now();

    if((current_time - contadorTiempo) > (ros::Duration)80 && bucle==1){     //Cada 30s, vamos cambiando los waypoints (va cambiando la racha de waypoints)
      bucle++;
      contadorTiempo = ros::Time::now();
    }

    if((current_time - contadorTiempo) > (ros::Duration)25 && bucle!=1){     //Cada 30s, vamos cambiando los waypoints (va cambiando la racha de waypoints)
      bucle++;
      contadorTiempo = ros::Time::now();
    }

    if(bucle>3)bucle = 1;
    //Publicar waypoints en "/PlannedPath"
    if(bucle == 1)waypoints_pub.publish(waypoints1);
    if(bucle == 2)waypoints_pub.publish(waypoints2);
    if(bucle == 3)waypoints_pub.publish(waypoints3);

    last_time = current_time;
    r.sleep();
  }
}