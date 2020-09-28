#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher datainput("/usv_comms/transceiver/data_input", &str_msg);

void messageCb( const std_msgs::String& toggle_msg){
  datainput.publish(&toggle_msg);   // blink the led
}

ros::Subscriber<std_msgs::String> sub("/usv_comms/station_transceiver/boat_data", &messageCb );

void setup()
{
  nh.initNode();
  nh.advertise(datainput);
  nh.subscribe(sub);

}

void loop()
{
  nh.spinOnce();
  delay(1000);
}
