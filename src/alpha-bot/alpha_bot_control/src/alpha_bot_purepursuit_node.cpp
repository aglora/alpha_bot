//NODO DE SEGUIMIENTO DE CAMINOS MEDIANTE EL ALGORITMO DE PURE PURSUIT

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>                        //Para convertir los cuaternios en rpy (y quedarnos solamente con yaw)
#include <nav_msgs/Odometry.h>            //Tipo de dato para la pose en cada instante del robot
#include <nav_msgs/Path.h>                //Tipo de dato de los waypoints
#include <geometry_msgs/Twist.h>          //Tipo de dato de cmd_vel (velocidad lineal + velocidad angular)
#include <geometry_msgs/PoseStamped.h>    //Tipo de dato dentro de los waypoints
#include <cmath>                          //Para el uso de las funciones "abs", "pow" y "fmod"
#include <iostream>                       //Para el uso de malloc
#include <cstdlib>                        //Para el uso de malloc
#include <vector>                         //Para "size()" => cálculo del número de elementos de un vector
#include <fstream>                        // Para ofstream: tratamiento de ficheros


/* PARÁMETROS (posible .yaml) */
#define PI 3.14159265358979323846264338327950288419716939937510582
#define LookAheadDistance 5               //Dada en número de waypoints hacia el final, desde el punto de mínima distancia
#define incT 0.75                         //Tiempo en el que se desea alcanzar el ángulo deseado por el Pure Pursuit
#define n_waypoints 1                    //Número de subwaypoints que se van a generar en las líneas rectas entre waypoints principales
#define umbralErrorDist 0.02              //Error máximo en posición para llegar al destino
#define vLinealDes 0.1                   //Velocidad a la que queremos que avance linealmente el robot mientras sigue el camino
#define wMax 1.00                         //Máxima velocidad de rotación durante el seguimiento del camino


/* VARIABLES GLOBALES */
double x,y,th;
nav_msgs::Path waypoints;
nav_msgs::Path waypointsAmpliados;
int indiceUltimoTramo = 0;
//double vLinealDes = 0.0;                  //Velocidad a la que queremos que avance linealmente el robot mientras sigue el camino
//double wMax = 0.0;                        //Máxima velocidad de rotación durante el seguimiento del camino

                     
/* FUNCIÓN DE LECTURA DE WAYPOINTS: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de waypoints del topic de waypoints, en el que publica el planificador
   de caminos 
   
   Estructura de un mensaje tipo waypoint:
   nav_msgs/Path.msg
    #An array of poses that represents a Path for a robot to follow
      std_msgs/Header header
      geometry_msgs/PoseStamped[] poses  <= **vector** de poses, CUIDADO: https://answers.ros.org/question/196840/nav_msgspath-without-the-field-pose/

   Enlaces de interés:
   http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html
   https://answers.ros.org/question/196840/nav_msgspath-without-the-field-pose/
   https://answers.ros.org/question/282094/publish-a-path/
   https://stackoverflow.com/questions/72358077/cannot-show-the-published-path-in-rviz-ros
   https://www.programcreek.com/cpp/?CodeExample=geometry+msgs+pose+stamped
*/

void waypointsCallback(const nav_msgs::Path& msg) {  
geometry_msgs::PoseStamped Pose_i;
int i,j;
geometry_msgs::Quaternion th_quat = tf::createQuaternionMsgFromYaw(th);
int flagIguales = 1;                   //El mensaje actual y el anterior son iguales hasta que se demuestre lo contrario
//Sólo implementaremos la funcionalidad original si descubrimos que el nuevo msg ES DISTINTO al que venía llegando en iteraciones anteriores => nos copiamos el nuevo vector de waypoints
for(i=0;i<waypoints.poses.size()-1 && !waypoints.poses.empty();i++){
  if(waypoints.poses[i+1].pose.position.x != msg.poses[i].pose.position.x || waypoints.poses[i+1].pose.position.y != msg.poses[i].pose.position.y){
    flagIguales=0;    //Waypoints[i+1] porque el 0 es la pose inicial del robot que la hemos añadido aparte 
    //std::cout << "DIFERENCIA: waypoints.poses[" << i+1 << "].position.x = " << waypoints.poses[i+1].pose.position.x << "  vs  msg.poses[" << i << "].position.x = " << msg.poses[i].pose.position.x << std::endl << std::endl << std::endl; 
  }
}

if(flagIguales == 0 || waypoints.poses.empty()){                                            //Realizamos la copia de msg si el mensaje nuevo es distinto al de la iteración anterior, o bien si el vector de
                                                                                            //waypoints anterior está vacío (primera iteración)
flagIguales = 1; //Como vamos a realizar una copia, los mensajes ahora ya sí son iguales
//Lecturas de waypoints del mensaje msg recibido del topic de waypoints "/PlannedPath"
//Destruir con clear todo el waypoints.poses
waypoints.poses.clear();  //Destrucción de "waypoints" en cada copia, queda reducido a "{}" 
waypoints.header.frame_id = msg.header.frame_id;     
//Waypoint[0] = pose en este instante <= comparando primero msg (nuevo) con waypoints (anterior). Si son distintos calculo el ampliado, pero como primer waypoint va a ser la pose en ese instante concreto del robot
Pose_i.pose.position.x = x;
Pose_i.pose.position.x = y;

Pose_i.pose.position.x = x;        
Pose_i.pose.position.y = y;          
Pose_i.pose.position.z = 0.0;

Pose_i.pose.orientation.x = 0.0;
Pose_i.pose.orientation.y = 0.0;
Pose_i.pose.orientation.z = th_quat.z;                                         
Pose_i.pose.orientation.w = th_quat.w;

waypoints.poses.push_back(Pose_i); 

//Vamos copiando el resto de waypoints
for(i=0;i<msg.poses.size();i++){
  Pose_i = msg.poses[i];
  waypoints.poses.push_back(Pose_i);     //Añadir nuevo waypoint como nueva fila del vector de waypoints "nav_msgs::Path waypoints"
}

//EXPANSIÓN DE LOS WAYPOINTS. CÁLCULO DE LOS SUBWAYPOINTS (NUEVOS WAYPOINTS) => OPERAMOS EN EL MARCO DE REFERENCIA GLOBAL
//Resultados de la expansión probados con código fuente auxiliar "CopiaPegaWaypointsPRUEBAS.cpp". Resultados: alpha_bot_control/Waypoints/"ExpansionWaypoints.png"
double incX=0.0, incY = 0.0, pend = 0.0;
waypointsAmpliados.header.frame_id = "map"; 
waypointsAmpliados.poses.clear();  //Destruir poses                       
for(i=0;i<waypoints.poses.size()-1;i++){                                                 //Llegamos hasta la -1 porque el waypoint final de todos los tramos lo metemos fuera de este bucle for
  waypointsAmpliados.poses.push_back(waypoints.poses[i]);                                //Meter el waypoint i en los waypoints ampliados. El waypoint i representa el punto inicial del tramo, y el i+1 el final
  incX = (waypoints.poses[i+1].pose.position.x-waypoints.poses[i].pose.position.x)/n_waypoints;
  incY = (waypoints.poses[i+1].pose.position.y-waypoints.poses[i].pose.position.y)/n_waypoints;
  pend = atan2((waypoints.poses[i+1].pose.position.y-waypoints.poses[i].pose.position.y),(waypoints.poses[i+1].pose.position.x-waypoints.poses[i].pose.position.x));
  geometry_msgs::Quaternion pend_quat = tf::createQuaternionMsgFromYaw(pend);

  for(j=0;j<n_waypoints-1;j++){                                                          //Meter los waypoints intermedios en los waypoints ampliados: el último no se tiene en cuenta porque ya es el punto final del tramo
      Pose_i.pose.position.x = waypoints.poses[i].pose.position.x + incX*(j+1);          //waypoints[i].x = poseRobot.x + inc_x_waypoint * (i + 1);
      Pose_i.pose.position.y = waypoints.poses[i].pose.position.y + incY*(j+1);          //WaypointIntermedio[j] 
      Pose_i.pose.position.z = 0.0;

      Pose_i.pose.orientation.x = 0.0;
      Pose_i.pose.orientation.y = 0.0;
      Pose_i.pose.orientation.z = pend_quat.z;                                           //Los subwaypoints intermedios tendrán como orientación la pendiente de la recta
      Pose_i.pose.orientation.w = pend_quat.w;
      waypointsAmpliados.poses.push_back(Pose_i);   
  }

}
waypointsAmpliados.poses.push_back(waypoints.poses[i]);                                  //Meter el waypoint final de todos los tramos en los waypoints ampliados
indiceUltimoTramo = waypointsAmpliados.poses.size()-n_waypoints-1;                       //Detectar último tramo del camino
}
}

/* FUNCIÓN DE LECTURA DE LA POSE: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   de la pose del robot 
   
   Enlaces de interés:
   http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
   https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
*/

void poseCallback(const nav_msgs::Odometry& msg) {  
//Lecturas de la pose del mensaje msg recibido del topic de ground truth /alpha_bot/mobile_base_controller/odom
geometry_msgs::Quaternion q0; q0.x = 0; q0.y = 0; q0.z = 0; q0.w = 0;   
x     = msg.pose.pose.position.x;
y     = msg.pose.pose.position.y;
//th    = tf::getYaw(msg.pose.pose.orientation);
if(msg.pose.pose.orientation == q0) th = 0;
else th    = tf::getYaw(msg.pose.pose.orientation);
//RECORTAR ÁNGULO th EN RADIANES AL INTERVALO [0,2*PI]
th = fmod(th,2*PI); //std::cout << "fmod = " << thDes << "  ";           //Resto de th/2*PI
if (th < 0)th += 2*PI;
}


/* FUNCIÓN DE LECTURA DE VELOCIDADES PARA PURE PURSUIT: 
   cada vez que es llamada en el bucle infinito del main, lee los valores
   vLinealDes y wMax para el Pure Pursuit 

*/

/*
void parametersCallback(const geometry_msgs::Twist& msg) {  
//Lecturas de parámetros del mensaje msg recibido del topic /purepursuit_parameters
vLinealDes = abs(msg.linear.x);
wMax = abs(msg.angular.z);
}*/


int main(int argc, char** argv){
  ros::init(argc, argv, "cmdvel_publisher");

  ros::NodeHandle n;
  ros::Subscriber waypoint_sub = n.subscribe("/PlannedPath", 50, waypointsCallback);                                /* Nombre del topic donde vamos a suscribirnos para obtener los waypoints */
  ros::Subscriber pose_sub = n.subscribe("odometry/filtered", 50, poseCallback);                                    /* Nombre del topic donde vamos a suscribirnos para obtener la pose actual del robot */
  //ros::Subscriber purepursuit_sub = n.subscribe("/purepursuit_parameters", 50, parametersCallback);                 /* Nombre del topic donde vamos a suscribirnos para obtener los parámetros para el Pure Pursuit */
  ros::Publisher cmdvel_pub  = n.advertise<geometry_msgs::Twist>("/alpha_bot/mobile_base_controller/cmd_vel", 50);  /* Nombre del topic donde vamos a publicar los comandos de velocidad para el robot */
  ros::Publisher waypoints_pub  = n.advertise<nav_msgs::Path>("/PlannedPathDOS", 50);                               /* Nombre del topic donde vamos a publicar el vector de waypoints ampliados */
  ros::Publisher odom_pub  = n.advertise<nav_msgs::Odometry>("/PoseMin", 50);                                       /* Nombre del topic donde vamos a publicar el punto del camino que se encuentra a mínima distancia del robot en cada instante */
  ros::Publisher odom2_pub = n.advertise<nav_msgs::Odometry>("/PuntoLookAhead", 50);                                /* Nombre del topic donde vamos a publicar el punto de Look Ahead del camino en cada instante */
  tf::TransformBroadcaster waypoint_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /* Variables locales de main */
  geometry_msgs::Twist velDes;             //Velocidad deseada que calculamos con el pure pursuit y que imprimiremos en cmd_vel ("/alpha_bot/mobile_base_controller/cmd_vel")
  double vDes = vLinealDes;                //Velocidad lineal de avance constante que llevará el robot siguiendo la trayectoria
  double wDes = 0.0;                       //Velocidad angular resultado del cálculo del algoritmo de Pure Pursuit
  double ekTh;                             //Error en ángulo del Pure Pursuit: ángulo que quiero - ángulo que tengo (pose)
  double thDes;                            //Ángulo deseado en el Pure Pursuit: ángulo que quiero 
  int i;                                   //Contador para recorrer bucles for
  //std::ofstream fich ("ExpControl1.txt",std::ios::app);  /* Variable fich para el manejo del fichero "<nombre_experimento>.txt"

  /* BUCLE INFINITO */
  ros::Rate r(50.0);
  while(n.ok()){

    ros::spinOnce();               
    current_time = ros::Time::now();
    
    //EXPANSIÓN DE LOS WAYPOINTS. CÁLCULO DE LOS SUBWAYPOINTS (NUEVOS WAYPOINTS)
    //Se hace en la rutina que lee los waypoints del topic de waypoints

    //ALGORITMO DE PURE PURSUIT
    //Encontrar waypoint situado a mínima distancia del robot
    //Recorrer todos los waypoints, calcular la distancia mínima a cada uno, y encontrar la distancia mínima y el índice del waypoint de distancia mínima
    int indiceMin = 0;
    double dist = 0.0, distMin = 1000.0;
    for(i=0; i<waypointsAmpliados.poses.size();i++){
      dist = sqrt(pow(abs(x-waypointsAmpliados.poses[i].pose.position.x),2)+pow(abs(y-waypointsAmpliados.poses[i].pose.position.y),2));
      if(dist < distMin){
        distMin = dist;
        indiceMin = i;
      }
    }  //Comprobado indiceMin se calcula bien. CUIDADO: direccionar el vector de poses requiere de un bucle for
    
    // OBTENCIÓN DEL WAYPOINT DE MÍNIMA DISTANCIA 
    // CONSTRUIR Y PUBLICAR PUNTO DE MÍNIMA DISTANCIA
    nav_msgs::Odometry PoseDistMin;
    PoseDistMin.header.frame_id = "map"; 
    for(i=0;i<waypointsAmpliados.poses.size();i++){
      if(i==indiceMin)PoseDistMin.pose.pose = waypointsAmpliados.poses[i].pose; 
    }

    //Elegir punto de Look Ahead como waypoint que se encuentre a un número de waypoints LookAheadDistance por delante del punto de mínima distancia en el vector de waypoints => punto destino
    //Partimos del waypoint de distancia mínima, y buscamos el waypoint que esté a LookAheadDistance waypoints por delante de él
    int indiceLookAhead = 0;
    for(i=indiceMin; i<waypointsAmpliados.poses.size(); i++){
      dist = sqrt(pow(abs(waypointsAmpliados.poses[i].pose.position.x-waypointsAmpliados.poses[indiceMin].pose.position.x),2)+pow(abs(waypointsAmpliados.poses[i].pose.position.y-waypointsAmpliados.poses[indiceMin].pose.position.y),2));
     /* if(dist < 0.9*LookAheadDistance && dist < 1.1*LookAheadDistance){   //Si la distancia desde el punto de mínima distancia hacia el waypoint i es LookAheadDistance (o dentro de un rango del 10%), este será el 
                                                                            //Punto de Look Ahead (punto al que deseo dirigirme)
        indiceLookAhead = i;
      }*/
      if(indiceMin + LookAheadDistance < waypointsAmpliados.poses.size()-1) indiceLookAhead = indiceMin + LookAheadDistance;
      else indiceLookAhead = waypointsAmpliados.poses.size()-1;             //El índice de Look Ahead siempre está LookAheadDistance waypoints por delante. Si ya hemos llegado al final del camino, entonces el
                                                                            //el punto de LookAhead es el punto final de la trayectoria
      //else if(indiceMin == waypointsAmpliados.poses.size()-1)indiceLookAhead = waypointsAmpliados.poses.size()-1;
    }

    // OBTENCIÓN DEL WAYPOINT DE LOOKAHEAD
    nav_msgs::Odometry PoseLookAhead;
    PoseLookAhead.header.frame_id = "map"; 
    for(i=indiceLookAhead;i<waypointsAmpliados.poses.size();i++){
      if(i==indiceLookAhead){
        PoseLookAhead.pose.pose = waypointsAmpliados.poses[i].pose; 
        break;
      }
    }  
    
    //Calcular ángulo deseado
    for(i=indiceLookAhead;i<waypointsAmpliados.poses.size();i++){
      if(i==indiceLookAhead){
        thDes = atan2((waypointsAmpliados.poses[i].pose.position.y-y),(waypointsAmpliados.poses[i].pose.position.x-x));
          //RECORTAR ÁNGULO thDes EN RADIANES AL INTERVALO [0,2*PI]
          thDes = fmod(thDes,2*PI); //std::cout << "fmod = " << thDes << "  ";           //Resto de thDes/2*PI
          if (thDes < 0)thDes += 2*PI;
          //std::cout << "th recortada = " << th*180/PI << "   " << "thDes recortada = " << thDes*180/PI << std::endl << std::endl;
        break;
      }
    }

    //Calcular error en ángulo y por tanto wDes
    ekTh = thDes - th;
    wDes = ekTh/incT;if(abs(wDes)>wMax)wDes = (wDes>0)*wMax+(wDes < 0)*(-wMax);   //Si la velocidad pasa de wMax en valor absoluto, saturarla a [-wMax,wMax]
    if(abs(ekTh)>PI)wDes = -wDes;

    //Reconstruir velocidad a publicar, velDes
    //Si estamos en el punto inicial del camino (si estamos comenzando un nuevo camino => el punto de mínima distancia coincide con el waypoint 0) => sólo nos giramos para orientarnos hacia el camino
    //Estando en el waypoint inicial, consideraremos que podemos avanzar cuando la orientación respecto al camino, ekTh, sea menor de 10º == 10*180/PI
    
    //Calculamos la distancia al punto final de todos los waypoints desde la pose actual del robot, de manera que nos detendremos cuando hayamos alcanzado el punto con un error de 1 cm
    double distFinal;
    for(i=waypointsAmpliados.poses.size()-1;i<waypointsAmpliados.poses.size();i++){
      if(i==waypointsAmpliados.poses.size()-1){
          distFinal = sqrt(pow(abs(x-waypointsAmpliados.poses[i].pose.position.x),2)+pow(abs(y-waypointsAmpliados.poses[i].pose.position.y),2));
          break;
      }
    }

    //Si estamos en el último tramo, el comportamiento del algoritmo cambia:
    //1. Calcular thDes como orientación necesaria para llegar desde el punto actual al punto objetivo final del camino


    if(indiceMin >= indiceUltimoTramo){

    for(i=waypointsAmpliados.poses.size()-1;i<waypointsAmpliados.poses.size();i++){
      if(i==waypointsAmpliados.poses.size()-1){
        if(distFinal>umbralErrorDist)thDes = atan2((waypointsAmpliados.poses[i].pose.position.y-y),(waypointsAmpliados.poses[i].pose.position.x-x));
        else thDes = tf::getYaw(waypointsAmpliados.poses[i].pose.orientation);
          //RECORTAR ÁNGULO thDes EN RADIANES AL INTERVALO [0,2*PI]
          thDes = fmod(thDes,2*PI); //std::cout << "fmod = " << thDes << "  ";          //Resto de thDes/2*PI
          if (thDes < 0)thDes += 2*PI;
          ekTh = thDes-th;
          wDes = ekTh/incT;if(abs(wDes)>wMax)wDes = (wDes>0)*wMax+(wDes < 0)*(-wMax);   //Si la velocidad pasa de wMax en valor absoluto, saturarla a [-wMax,wMax]
          if(abs(ekTh)>PI)wDes = -wDes;
        break;
      }
    }

      //2. Si el error en ángulo respecto al punto final es mayor de 5º, nos detenemos y sólo nos reorientamos. Si estamos encaminados, podemos avanzar
      if(abs(ekTh) <= 5.0*PI/180.0)vDes = 0.5*(distFinal-0.01);    
      else vDes = 0.0;
    }
    else vDes = vLinealDes;


    //Imprimir la v y la w calculadas en cmd_vel
    if(indiceMin == 0 && (abs(ekTh) >= 10.0*PI/180.0)){    //La velocidad también se hace cero cuando hemos llegado al punto final de la trayectoria (con un determinado umbral de error)
      velDes.linear.x = 0.0;
      velDes.linear.y = 0.0;
      velDes.linear.z = 0.0;
    }
    else{
      velDes.linear.x = vDes;
      velDes.linear.y = vDes;
      velDes.linear.z = 0.0;
    }
    //std::cout << "indiceLookAhead || waypointsAmpliados.poses.size() || distFinal || umbralErrorDist   " << indiceLookAhead << "||" << waypointsAmpliados.poses.size() << "||" << distFinal << "||" << umbralErrorDist << std::endl << std::endl;
    //Se dan en x,y,z   suponemos que se refiere a r,p,y
    velDes.angular.x = 0.0;
    velDes.angular.y = 0.0;
    velDes.angular.z = wDes;

    /* Guardar datos de la iteración actual en el fichero "fich" */            
    // if(!fich)printf("Imposible abrir el fichero *.txt* \n");
    // else fich << ros::Time::now() << "," << indiceMin << "," << distMin << "," << x << "," << y  << "," << th << "," << thDes << "," << PoseDistMin.pose.pose.position.x << "," << PoseDistMin.pose.pose.position.y << "," << vLinealDes << "," << wDes << "," << LookAheadDistance << ";" << std::endl; //Si el fichero es accesible, escribir en él

    //Publicar waypoints en "/PlannedPathDOS", y los puntos de mínima distancia y de Look Ahead en cada instante
    waypoints_pub.publish(waypointsAmpliados);         //Funciona porque publico los nuevos waypoints sólo cuando ya los he escrito, no cuando los borré con clear
    odom_pub.publish(PoseDistMin);
    odom2_pub.publish(PoseLookAhead);

    //Publicar acción de control deseada en /cmd_vel Y RETIRARLA DEL SELECTOR DE VELOCIDADES MANUAL
    cmdvel_pub.publish(velDes);

    last_time = current_time;
    r.sleep();
  }
}
