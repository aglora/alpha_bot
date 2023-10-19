#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>                     //Para convertir los cuaternios en rpy (y quedarnos solamente con yaw)
#include <nav_msgs/Odometry.h>         //Tipo de dato para la pose en cada instante del robot
#include <nav_msgs/OccupancyGrid.h>    //Tipo de dato para el costmap
#include <nav_msgs/Path.h>             //Tipo de dato de los waypoints
#include <geometry_msgs/Twist.h>       //Tipo de dato de cmd_vel (velocidad lineal + velocidad angular)
#include <geometry_msgs/PoseStamped.h> //Tipo de dato dentro de los waypoints
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream> // Para ofstream: tratamiento de ficheros
#include <chrono> // Para contabilizar tiempo de ejecución
#include <string>
#include <sensor_msgs/LaserScan.h>


#define lethal_cost 100

// Estructuras de datos
struct coord
{
   int x;
   int y;
};
struct node
{
   int id;
   struct coord coordenadas;
   int g_cost;
   int parent_node;
};

/* VARIABLES GLOBALES */
// Mensajes

nav_msgs::Odometry poseRobot; // Pose del robot, viene de localization
nav_msgs::Odometry poseObjetivo;
nav_msgs::Path shortPath;
move_base_msgs::MoveBaseActionGoal goal;
nav_msgs::OccupancyGrid GridCostmap;

// Djisktra
int dimX, dimY;
int8_t **costmap;   // Mapa de costes obtenido del costmap_2d
float **trajectory; // Trayectoria generada
struct node **nodes;
struct node initial_node;
struct node current_node;
struct node neighbor_node;
struct node goal_node;
std::vector<node> open_list;
std::vector<node> closed_list;
int min_g_cost;
int indice_min_g_cost;
int x_neighbor;
int y_neighbor;
bool flagFoundClosedList;
bool flagFoundOpenList;
std::vector<int> path_id;
float cornerReducedX, cornerReducedY;

// Subscriptores y publicadores
ros::Subscriber costmap_sub;
ros::Subscriber goal_sub;
ros::Subscriber localization_sub;
ros::Subscriber laser_sub;
ros::Publisher plannedPath_pub;
ros::Publisher costmap_pub;

//Experimento
//std::ofstream ficheroDatosExp("//home/lion/ros_workspaces/alpha_bot_ws/src/alpha-bot/alpha_bot_planner/src/datosExp.txt",std::ios::app); //fichero datos experimento en formato CSV
//float minRange=-1;

void actualiza_nodo(int x_neighbor, int y_neighbor, bool diagonal)
{
   if (x_neighbor >= 0 && x_neighbor < dimX && y_neighbor < dimY && y_neighbor >= 0)
   {
      if (costmap[y_neighbor][x_neighbor] < lethal_cost) // si es vecino (no obstáculo)
      {
         neighbor_node.id = nodes[y_neighbor][x_neighbor].id;
         neighbor_node.g_cost = current_node.g_cost + costmap[y_neighbor][x_neighbor] + 10;
         if (diagonal)
            neighbor_node.g_cost += 4;
         flagFoundClosedList = false;
         for (int i = 0; i < closed_list.size(); i++) // buscar nodo en la lista cerrada
         {
            if (neighbor_node.id == closed_list[i].id)
            {
               flagFoundClosedList = true;
               break;
            }
         }
         if (flagFoundClosedList == false) // si no se encuentra en la lista cerrada
         {
            flagFoundOpenList = false;
            for (int i = 0; i < open_list.size(); i++) // buscar nodo en la lista abierta
            {
               if (neighbor_node.id == open_list[i].id)
               {
                  flagFoundOpenList = true;
                  break;
               }
            }
            if (flagFoundOpenList == true) // si está dentro de la lista abierta
            {
               if (neighbor_node.g_cost < nodes[y_neighbor][x_neighbor].g_cost) // actualizar parent y g_cost si este último es menor que el actual
               {
                  nodes[y_neighbor][x_neighbor].g_cost = neighbor_node.g_cost;
                  neighbor_node.parent_node = current_node.id;
               }
            }
            else // si no está dentro de la lista abierta
            {
               neighbor_node.parent_node = current_node.id;
               open_list.push_back(neighbor_node);
            }
         }
      }
   }
}

void goalCallback(const move_base_msgs::MoveBaseActionGoal &msg)
{
   goal = msg;


   auto start = std::chrono::high_resolution_clock::now(); //empieza a contar tiempo de cálculo de ruta

   dimX = GridCostmap.info.width;
   dimY = GridCostmap.info.height;
   costmap = new int8_t *[dimY];
   for (int i = 0; i < dimY; i++)
      costmap[i] = new int8_t[dimX];
   // Relleno el mapa de costes con el mensaje recibido, haciéndolo matriz en vez de vector
   for (int i = 0; i < dimY; i++)
   {
      for (int j = 0; j < dimX; j++)
      {
         costmap[i][j] = GridCostmap.data[j + i * dimX];
      }
   }

   nodes = new struct node *[dimY];
   for (int i = 0; i < dimY; i++)
      nodes[i] = new struct node[dimX];

   for (int i = 0; i < dimY; i++)
   {
      for (int j = 0; j < dimX; j++)
      {
         nodes[i][j].id = i * dimX + j;
         nodes[i][j].coordenadas = {j, i};
         nodes[i][j].g_cost = -1;
         nodes[i][j].parent_node = -1;
      }
   }

   initial_node.id = (int)((poseRobot.pose.pose.position.x - cornerReducedX) / GridCostmap.info.resolution) + (int)((poseRobot.pose.pose.position.y - cornerReducedY) / GridCostmap.info.resolution) * dimX;
   initial_node.coordenadas = {(int)((poseRobot.pose.pose.position.x - cornerReducedX) / GridCostmap.info.resolution), (int)((poseRobot.pose.pose.position.y - cornerReducedY) / GridCostmap.info.resolution)};
   initial_node.g_cost = 0;
   initial_node.parent_node = -1;
   open_list.push_back(initial_node);
   std::cout << "xIni:" << initial_node.coordenadas.x << " yIni:" << initial_node.coordenadas.y << " idIni:" << initial_node.id << std::endl;

   goal_node.id = (int)((goal.goal.target_pose.pose.position.x - cornerReducedX) / GridCostmap.info.resolution) + (int)((goal.goal.target_pose.pose.position.y - cornerReducedY) / GridCostmap.info.resolution) * dimX;
   goal_node.coordenadas = {(int)((goal.goal.target_pose.pose.position.x - cornerReducedX) / GridCostmap.info.resolution), (int)((goal.goal.target_pose.pose.position.y - cornerReducedY) / GridCostmap.info.resolution)};
   goal_node.g_cost = -1;
   goal_node.parent_node = -1;
   std::cout << "xG:" << goal_node.coordenadas.x << " yG:" << goal_node.coordenadas.y << " idG:" << goal_node.id << std::endl;

// fase 1
FASE_1:
   while (open_list.size() > 0)
   {
      min_g_cost = open_list[0].g_cost;
      indice_min_g_cost = 0;
      for (int i = 1; i < open_list.size(); i++)
      {
         if (min_g_cost > open_list[i].g_cost)
         {
            min_g_cost = open_list[i].g_cost;
            indice_min_g_cost = i;
         }
      }
      current_node.id = open_list[indice_min_g_cost].id;
      current_node.coordenadas.x = (current_node.id) % dimX;
      current_node.coordenadas.y = (int)(current_node.id + 1) / (int)dimX;
      current_node.g_cost = open_list[indice_min_g_cost].g_cost;
      current_node.parent_node = open_list[indice_min_g_cost].parent_node;
      open_list.erase(open_list.begin() + indice_min_g_cost);
      closed_list.push_back(current_node);
      if (current_node.id == goal_node.id)
      {
         goto FASE_2;
      }

      /*-------UPPER---------------*/
      x_neighbor = current_node.coordenadas.x;
      y_neighbor = current_node.coordenadas.y - 1;
      actualiza_nodo(x_neighbor, y_neighbor, 0);

      /*-------LOWER---------------*/
      x_neighbor = current_node.coordenadas.x;
      y_neighbor = current_node.coordenadas.y + 1;
      actualiza_nodo(x_neighbor, y_neighbor, 0);

      /*-------RIGHT---------------*/
      x_neighbor = current_node.coordenadas.x + 1;
      y_neighbor = current_node.coordenadas.y;
      actualiza_nodo(x_neighbor, y_neighbor, 0);

      /*-------LEFT---------------*/
      x_neighbor = current_node.coordenadas.x - 1;
      y_neighbor = current_node.coordenadas.y;
      actualiza_nodo(x_neighbor, y_neighbor, 0);

      /*-------UPPER-LEFT CORNER---------------*/
      x_neighbor = current_node.coordenadas.x - 1;
      y_neighbor = current_node.coordenadas.y - 1;
      actualiza_nodo(x_neighbor, y_neighbor, 1);

      /*-------UPPER-RIGHT CORNER---------------*/
      x_neighbor = current_node.coordenadas.x + 1;
      y_neighbor = current_node.coordenadas.y - 1;
      actualiza_nodo(x_neighbor, y_neighbor, 1);

      /*-------LOWER-LEFT CORNER---------------*/
      x_neighbor = current_node.coordenadas.x - 1;
      y_neighbor = current_node.coordenadas.y + 1;
      actualiza_nodo(x_neighbor, y_neighbor, 1);

      /*-------LOWER-RIGHT CORNER---------------*/
      x_neighbor = current_node.coordenadas.x + 1;
      y_neighbor = current_node.coordenadas.y + 1;
      actualiza_nodo(x_neighbor, y_neighbor, 1);
   }
   // si se alcanza esta parte de código significa que no se ha podido encontrar un camino
   std::cout << "Camino no encontrado" << std::endl;
   goto END;

FASE_2:
   current_node = closed_list.back();
   while (1)
   {
      path_id.push_back(current_node.id);
      if (current_node.id == initial_node.id)
      {
         break;
      }
      current_node.id = current_node.parent_node;
      // buscar nodo en lista cerrada para actualizar parent
      for (int i = 0; i < closed_list.size(); i++)
      {
         if (current_node.id == closed_list[i].id)
         {
            current_node = closed_list[i];
            break;
         }
      }
   }
   std::reverse(path_id.begin(), path_id.end());
   std::cout << "Camino encontrado" << std::endl;
   std::cout << "tamano: " << path_id.size() << std::endl;
   for (int i = 0; i < path_id.size(); i++)
      std::cout << path_id[i] << std::endl;

   shortPath.header.frame_id = "map";
   shortPath.poses.resize(path_id.size());
   std::cout << "----------------" << std::endl;
   for (int i = 0; i < path_id.size(); i++)
   {
      shortPath.poses[i].pose.position.x = (path_id[i] % dimX) * GridCostmap.info.resolution + cornerReducedX;
      shortPath.poses[i].pose.position.y = ((int)(path_id[i] + 1) / (int)dimX) * GridCostmap.info.resolution + cornerReducedY;
      std::cout << "xPath: " << shortPath.poses[i].pose.position.x << " yPath: " << shortPath.poses[i].pose.position.y << std::endl;
   }
   for (int i = 0; i < path_id.size() - 1; i++)
   {
      shortPath.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(atan2(shortPath.poses[i + 1].pose.position.y - shortPath.poses[i].pose.position.y, shortPath.poses[i + 1].pose.position.x - shortPath.poses[i].pose.position.x));
   }
   shortPath.poses[path_id.size() - 1].pose.orientation = goal.goal.target_pose.pose.orientation;

   plannedPath_pub.publish(shortPath);

END:
   auto end = std::chrono::high_resolution_clock::now(); //finaliza contando tiempo de planificación del algoritmo
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);

   //ficheroDatosExp << std::to_string(minRange) << ","<< std::to_string(duration.count())<< std::endl;
   

   open_list.clear();
   closed_list.clear();
   path_id.clear();
   for (int i = 0; i < dimY; i++)
      delete[] nodes[i];
   delete[] nodes;
}

void costmapCallback(const nav_msgs::OccupancyGrid &msg)
{
   int xMin = 3999, yMin = 3999;
   int xMax = 0, yMax = 0;
   std::cout << "Tamano costmap vector: " << msg.data.size() << std::endl;
   std::cout << "altura costmap vector: " << msg.info.height << std::endl;
   std::cout << "ancho costmap vector: " << msg.info.width << std::endl;
   for (int i = 0; i < msg.info.height; i++)
   {
      for (int j = 0; j < msg.info.width; j++)
      {
         // std::cout<<"indice: "<<j+i*GridCostmap.info.width<<std::endl;
         if (msg.data[j + i * msg.info.width] != 0)
         {
            if (j < xMin)
            {
               xMin = j;
            }
            if (i < yMin)
            {
               yMin = i;
            }
            if (j > xMax)
            {
               xMax = j;
            }
            if (i > yMax)
            {
               yMax = i;
            }
         }
      }
   }
   for (int i = yMin; i < yMax; i++)
   {
      for (int j = xMin; j < xMax; j++)
      {
         GridCostmap.data.push_back(msg.data[j + i * msg.info.width]);
         // std::cout << " " << (int)GridCostmap.data[GridCostmap.data.size() - 1] << ", ";
      }
      std::cout << std::endl;
   }
   std::cout << "xMin: " << xMin << "yMin: " << yMin << "xMax: " << xMax << "yMax: " << yMax << std::endl;
   GridCostmap.header.frame_id = "base_link";
   GridCostmap.info.resolution = msg.info.resolution;
   GridCostmap.info.height = yMax - yMin;
   GridCostmap.info.width = xMax - xMin;
   float cornerX = -(msg.info.width / 2.0) * msg.info.resolution, cornerY = -(msg.info.height / 2.0) * msg.info.resolution; // bottom right
   std::cout << "resolution" << msg.info.resolution << " width" << GridCostmap.info.width << " height" << GridCostmap.info.height << std::endl;
   std::cout << "cornerX: " << cornerX << "cornerY: " << cornerY << std::endl;
   // esquinas costmap reducido respecto al centro mapa (origen robot)
   cornerReducedX = cornerX + xMin * msg.info.resolution;
   cornerReducedY = cornerY + yMin * msg.info.resolution;
   std::cout << "cornerReducedX: " << cornerReducedX << "cornerReducedY: " << cornerReducedY << std::endl;
}

void localizationCallback(const nav_msgs::Odometry &msg)
{
   poseRobot = msg;
   // std::cout<<poseRobot.pose.pose.position<<std::endl;
}

// void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//   // Acceder a los datos del mensaje aquí
//   float angle_min = msg->angle_min;
//   float angle_max = msg->angle_max;
//   float angle_increment = msg->angle_increment;
//   std::vector<float> ranges = msg->ranges;

//   for (int i = 0; i < ranges.size(); i++) {
//     float angle = angle_min + i * angle_increment;
//     if((ranges[i]<minRange || minRange==-1) )
//       minRange = ranges[i];
//   }
// }

int main(int argc, char **argv)
{
   ros::init(argc, argv, "global_planner");
   ros::NodeHandle n;
   //ficheroDatosExp.open("datosExp.txt");
   //if(ficheroDatosExp.is_open()!=1) { while(1) std::cout<< "No pudo abrirse el fichero"<<std::endl; }
   costmap_sub = n.subscribe("/move_base/global_costmap/costmap", 50, costmapCallback);
   costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/costmap_reduced", 50);
   localization_sub = n.subscribe("/odometry/filtered", 50, localizationCallback);
   //laser_sub = n.subscribe("/alpha_bot/scan", 50, laserCallback);
   goal_sub = n.subscribe("/move_base/goal", 50, goalCallback);
   plannedPath_pub = n.advertise<nav_msgs::Path>("/PlannedPath", 50);

   /* BUCLE INFINITO */
   ros::Rate r(50.0);
   while (n.ok())
   {
      ros::spinOnce();
      costmap_pub.publish(GridCostmap);
      r.sleep();
   }
   for (int i = 0; i < dimY; i++)
      delete[] costmap[i];
   delete[] costmap;
   //ficheroDatosExp.close();
}
