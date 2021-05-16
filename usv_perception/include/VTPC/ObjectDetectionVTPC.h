/** ----------------------------------------------------------------------------
 * @file:     ObjectDetectionVTPC.cpp
 * @date:     January 11, 2021
 * @coauthor: Rodolfo Cuan
 * @email:    a01233155@itesm.mx
 * 
 * @brief:  Contains the description of the class use for object detection in
 *          VantTec Point Cloud (VTPC) class. 
 * ---------------------------------------------------------------------------*/

// IFNDEF ----------------------------------------------------------------------
#ifndef ObjectDetectionVTPC_H_
#define ObjectDetectionVTPC_H_


// NAMESPACES ------------------------------------------------------------------

using namespace std; 

// INCLUDES --------------------------------------------------------------------

#include <vector> 
#include <iostream>
#include <algorithm>
#include <pcl/point_types.h>

// CLASS ----------------------------------------------------------------------

class gridObj{

public:
    bool occupy;
    int density;
    int groupNum;
    pcl::PointXYZ centerPoint;

    pcl::PointXYZ pt_min_x, pt_min_y, pt_max_x, pt_max_y; 
    pcl::PointXYZ pt_min, pt_max;

    std::vector<int> indices;
    Eigen::Vector3f voxel_min, voxel_max;

    //Eigen::Vector4f pt_min, pt_max;

    inline gridObj(){
        occupy = false;
        density = 0;
        groupNum = 0;
        indices.clear();

    }

    gridObj operator+(const gridObj& b){
        gridObj res;
        res.density = this->density + b.density;        
    
        res.pt_max.z = max(this->pt_max.z , b.pt_max.z);
        res.pt_min.z = min(this->pt_min.z , b.pt_min.z);
        res.pt_max.y = max(this->pt_max.y , b.pt_max.y);
        res.pt_min.y = min(this->pt_min.y , b.pt_min.y);
        res.pt_max.x = max(this->pt_max.x , b.pt_max.x);
        res.pt_min.x = min(this->pt_min.x , b.pt_min.x);

        res.indices.insert(res.indices.end(),this->indices.begin(), this->indices.end());
        res.indices.insert(res.indices.end(),b.indices.begin(), b.indices.end());

        res.voxel_min = {min(this->voxel_min(0) , b.voxel_min(0)),
            min(this->voxel_min(1) , b.voxel_min(1)),
            min(this->voxel_min(2) , b.voxel_min(2))};
        res.voxel_max = {max(this->voxel_max(0) , b.voxel_max(0)),
            max(this->voxel_max(1) , b.voxel_max(1)),
            max(this->voxel_max(2) , b.voxel_max(2))};

        res.occupy = true;

        res.centerPoint.x = (res.voxel_max(0) + res.voxel_min(0)) /2;
        res.centerPoint.y = (res.voxel_max(1) + res.voxel_min(1)) /2;
        res.centerPoint.z = (res.pt_max.z + res.pt_min.z)/2;

        res.groupNum = this->groupNum;

        return res;
    }

    void display(){
        cout<<"ind max: "<<this->pt_max<<endl;
        cout<<"ind min: "<<this->pt_min<<endl;
        cout<<"voxel_min: "<<this->voxel_min(0)<<" "<<this->voxel_min(1)<<" "<<this->voxel_min(2)<<endl;
        cout<<"voxel_max: "<<this->voxel_max(0)<<" "<<this->voxel_max(1)<<" "<<this->voxel_max(2)<<endl;
        cout<<"groupNum: "<<this->groupNum<<endl;
        cout<<"density: "<<this->density<<endl;
    }

}; 

#endif


