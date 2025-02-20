#include "m5.h"

M5::M5(){
    id = 5;
    re_init();
}

USVOutput M5::update(const Eigen::Vector3f &pose, const  USVUpdate &params){
    outMsg.goals.clear();
    Eigen::Vector3f goal;
    this->pose = pose;

        switch(outMsg.state){
            case 0: // Initial state, look for first boat to travel to 
                goal = get_goal(params.obs_list);
                if(goal.norm() > 0.01){
                    outMsg.state = 1;
                    outMsg.goals = pack_goal(pose, goal, 1.);
                    last_goal = outMsg.goals[outMsg.goals.size() - 1];
                }
                break;
            case 1: // Travel to first boat
            case 2: // Shooting  the boat
            case 3: // Returning to original position
        }
}