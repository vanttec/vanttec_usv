#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>

class SetState{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_state = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    ros::Subscriber sub_state = nh.subscribe("/vectornav/ins_2d/NED_pose", 1000, &SetState::stateCallback, this);
    tf2::Quaternion q;
    gazebo_msgs::ModelState model_msg;

public:
    void stateCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
};

void SetState::stateCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    model_msg.model_name = "vtec_s3";
    
    // NED to ENU
    q.setRPY(0,0,-msg->theta);

    model_msg.pose.position.x = msg->x;
    model_msg.pose.position.y = -msg->y;
    model_msg.pose.position.z = 0;

    model_msg.pose.orientation.x = q.x();
    model_msg.pose.orientation.y = q.y();
    model_msg.pose.orientation.z = q.z();
    model_msg.pose.orientation.w = q.w();

    pub_state.publish(model_msg);
}

int main(int argc, char **argv){
    ros::init(argc,argv, "gazebo_interface");
    SetState state;
    ros::Rate rate(100);

    while (ros::ok){
        rate.sleep(); // wait 10 Hz
        ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time
    }

    return 0;
}