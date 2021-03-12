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
    float maxZ;
    float minZ;
    int groupNum;
    pcl::PointXYZ centerPoint;
    std::vector<int> indices;
    Eigen::Vector3f voxel_min, voxel_max;

    inline gridObj(){
        occupy = false;
        density = 0;
        maxZ = 0;
        minZ = 0;
        groupNum = 0;
        indices.clear();

    }

    gridObj operator+(const gridObj& b){
        gridObj res;
        res.density = this->density + b.density;
        res.maxZ = max(this->maxZ , b.maxZ);
        res.minZ = min(this->minZ , b.minZ);

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
        res.centerPoint.z = (res.maxZ + res.minZ)/2;

        res.groupNum = this->groupNum;

        return res;
    }

    void display(){
        cout<<"MaxZ: "<<this->maxZ<<endl;
        cout<<"minZ: "<<this->minZ<<endl;
        cout<<"voxel_min: "<<this->voxel_min(0)<<" "<<this->voxel_min(1)<<" "<<this->voxel_min(2)<<endl;
        cout<<"voxel_max: "<<this->voxel_max(0)<<" "<<this->voxel_max(1)<<" "<<this->voxel_max(2)<<endl;
        cout<<"groupNum: "<<this->groupNum<<endl;
        cout<<"density: "<<this->density<<endl;
    }

}; 

#endif


