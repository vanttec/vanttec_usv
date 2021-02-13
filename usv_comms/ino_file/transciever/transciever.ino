#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher boatConfirmation("/usv_comms/transceiver/boat_confirmation", &str_msg);
ros::Publisher boatStatus("/usv_comms/transceiver/boat_status", &str_msg);
ros::Publisher stationCommands("/usv_comms/transceiver/station_commands", &str_msg);

void messageConfirm( const std_msgs::String& toggle_msg){
  boatConfirmation.publish(&toggle_msg);   // blink the led
}

void messageStatus( const std_msgs::String& toggle_msg){
  boatStatus.publish(&toggle_msg);   // blink the led
}

void messageCommands( const std_msgs::String& toggle_msg){
  stationCommands.publish(&toggle_msg);   // blink the led
}
ros::Subscriber<std_msgs::String> confirmation("/usv_comms/boat_transceiver/course_confirmation", &messageConfirm );
ros::Subscriber<std_msgs::String> status("/usv_comms/boat_transceiver/boat_status", &messageStatus );
ros::Subscriber<std_msgs::String> commands("/usv_comms/station_transceiver/station_commands", &messageCommands );

void setup()
{
  nh.initNode();
  nh.advertise(boatConfirmation);
  nh.advertise(boatStatus);
  nh.advertise(stationCommands);

  nh.subscribe(confirmation);
  nh.subscribe(status);
  nh.subscribe(commands);
}

void loop()
{
  nh.spinOnce();
  delay(1000);
}
