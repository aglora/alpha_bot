#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String& msg) {
  ROS_INFO("I heard '%s'", msg.data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wait_for_publishers");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1, callback);
  ros::Publisher pub = nh.advertise<std_msgs::String>("talk", 1);
  std_msgs::String msg;
  msg.data = "Hello";

  while(ros::ok() && sub.getNumPublishers() == 0) {
    ros::Duration(0.2).sleep();
    ROS_INFO_THROTTLE(1.0, "Waiting for publishers...");
  }

  ROS_INFO("There are now %d subscribers connected!", sub.getNumPublishers());

  ros::Rate rate(1);
  while(ros::ok()) {
    ros::spinOnce();
    pub.publish(msg);
    rate.sleep();
  }

  return 0;
}


