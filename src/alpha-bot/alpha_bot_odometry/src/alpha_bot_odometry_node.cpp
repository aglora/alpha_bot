#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>                          //Para el uso de la función "abs"
#include <random>

/* VARIABLES GLOBALES */
double vx, vy, vth;

/* FUNCIÓN DE LECTURA DE VELOCIDADES: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de velocidad del ground truth  del robot y les añade ruido gaussiano, 
   simulando la lectura de un encoder con ruido
   
   Enlaces de interés:
   http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
   http://coliru.stacked-crooked.com/a/ade6e6cb7b606610
    */


void cmdvelCallback(const nav_msgs::Odometry& msg) {  

// Definir generador de números aleatorios según distribución gaussiana, con un error de entre 0 y 1
const double mean = 0.0;
const double stddev = 0.20;
auto dist = std::bind(std::normal_distribution<double>{mean, stddev},std::mt19937(std::random_device{}()));

//Lecturas de velocidades del mensaje msg recibido del topic de ground truth /alpha_bot/mobile_base_controller/odom
vx     = msg.twist.twist.linear.x;
vy     = msg.twist.twist.linear.y;
vth    = msg.twist.twist.angular.z;

//Añadir el ruido gaussiano sólo si el robot está en movimiento (la odometría no provoca derivas si el vehículo está totalmente detenido (si todas las velocidades son 0))
//if((vx!=0) || (vy!=0) || (vth!=0)){
if((abs(vx)>=0.01) || (abs(vy)>=0.01) || (abs(vth)>=0.01)){   //La velocidad tiene pequeñas variaciones numéricas del orden de +-10^(-7) y puntualmente como maximo del orden de 
                                                              //alrededir de +-0.01 => si es superior a esto, consideramos que hay movimiento => los encoders introducen error
  vx  += dist();
  vy  += dist();
  vth += dist();
}


}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/alpha_bot/mobile_base_controller/odom", 50, cmdvelCallback);     /* Nombre del topic donde vamos a suscribirnos */
  ros::Publisher odom_pub  = n.advertise<nav_msgs::Odometry>("wheel_odometry", 50);                       /* Nombre del topic donde vamos a publicar */
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);
  while(n.ok()){

    ros::spinOnce();               
    current_time = ros::Time::now();

    //Odometría del robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //Publicar las transformaciones entre frames sobre tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //Enviar la transformación
    //odom_broadcaster.sendTransform(odom_trans); /* No la enviamos porque hace que mapa se vuelva inestable */

    //Reconstruir la odometría como un mensaje nav_msgs/Odometry
    nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";  

    //Actualizar la pose
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //Actualizar las velocidades
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //Publicar odometría en /wheel_odometry
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}