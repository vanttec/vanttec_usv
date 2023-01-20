#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

//Subscribers variables declaration
String data_input = "";
String usv_master_status = "";
String message_xbee  = ""; 

void data_callback(const std_msgs::String& cmd_msg){
    data_input = cmd_msg.data;
    //rospy.loginfo('[USV] Sending data: ', self.boat_data)
    //#self.device.send_data_async(self.remote_device, self.boat_data)
}
void usv_master_callback(const std_msgs::String& cmd_msg){
    usv_master_status = cmd_msg.data;
}
void message_xbee_callback(const std_msgs::String& cmd_msg){
    message_xbee = cmd_msg.data;
}

std_msgs::Empty stop;
std_msgs::String course;
std_msgs::Empty start;
std_msgs::String general;
ros::Subscriber<std_msgs::String> data("/usv_comms/boat_transceiver/data_input", data_callback);
ros::Subscriber<std_msgs::String> status("/usv_master/usv_master_status", usv_master_callback);
ros::Subscriber<std_msgs::String> message("/message_xbee", message_xbee_callback);
ros::Publisher stop_pub("/usv_comms/boat_transceiver/stop_mission", &stop);
ros::Publisher course_pub("/usv_comms/boat_transceiver/course_config", &course);
ros::Publisher start_pub("/usv_comms/boat_transceiver/start_mission", &start);
ros::Publisher general_status_pub("/usv_comms/boat_transceiver/general_status", &general);

void setup() {
  //ROS
  nh.initNode();
  nh.subscribe(data);
  nh.subscribe(status);
  nh.subscribe(message);
  nh.advertise(stop_pub);
  nh.advertise(course_pub);
  nh.advertise(start_pub);
  nh.advertise(general_status_pub);
}

void loop() {
  if(message_xbee == "s" || message_xbee == "S"){
    start.data = "start"; 
    start_pub.publish( &start );
    message_xbee = "START";
  }
  else if(message_xbee == "k" || message_xbee == "K"){
    stop.data = "STOP"; 
    stop_pub.publish( &stop );
    message_xbee = "STOP";
  }
  course_pub.publish( &course );
  nh.spinOnce();
  delay(1);
}
