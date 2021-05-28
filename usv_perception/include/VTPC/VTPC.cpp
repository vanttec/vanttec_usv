/** ----------------------------------------------------------------------------
 * @file:     VTPC.cpp
 * @date:     Octubre 8, 2020
 * @coauthor: Ivana Collado
 * @email:    a00569475@itesm.mx
 * @coauthor: Rodolfo Cuan
 * @email:    a01233155@itesm.mx
 * 
 * @brief: Contains the implementations of the  VantTec Point Cloud (VTPC) class. 
 *         See the header file for more information.
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include "VTPC.h"

// CLASS ----------------------------------------------------------------------
template<class PointType>
VTPC<PointType>::VTPC(){
  //Initialize Pointer 
  cloudPtr_ = typename pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  viewer_ = NULL;

  groupNumCnt_ = 0;
  isFirstIteration_ = false;

} 

template<class PointType>
VTPC<PointType>::~VTPC() {
//Empty body
}

// FUNCTIONS -------------------------------------------------------------------
template<class PointType>
void VTPC<PointType>::SetCloud(const pcl::PointCloud<PointType> &in_cloud) {
  *cloudPtr_ = in_cloud;
}

template<class PointType>
pcl::PointCloud<PointType> VTPC<PointType>::GetCloud() {
  return *cloudPtr_;
}

template<class PointType>
sensor_msgs::PointCloud2 VTPC<PointType>::GetCloudMsg() {
  sensor_msgs::PointCloud2 output_msg;
  pcl::PCLPointCloud2 point_cloud;

  //from PCL to PointCloud2
  pcl::toPCLPointCloud2(*cloudPtr_, point_cloud); 
  //from PointCloud2 to sensor_msgs::PointCloud2	
  pcl_conversions::fromPCL(point_cloud,output_msg);

  return output_msg;
}

template<class PointType>
void VTPC<PointType>::PassThrough(const std::string &dim,const float &minLim,
                                  const float &maxLim){
  pcl::PassThrough<PointType> pass;

  pass.setInputCloud(cloudPtr_);
  pass.setFilterFieldName (dim);
  pass.setFilterLimits (minLim, maxLim);
  pass.filter(*cloudPtr_);
  
}

template<class PointType>
void VTPC<PointType>::CropBox(const float &minX, const float &maxX, const float &minY,
                              const float &maxY, const float &minZ, const float &maxZ){
  pcl::CropBox<PointType> boxFilter;

  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxFilter.setInputCloud(cloudPtr_);
  boxFilter.filter(*cloudPtr_);
  
}

template<class PointType>
void VTPC<PointType>::DownsampleVoxelGrid(const float &leaf_size){
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud(cloudPtr_);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloudPtr_);

}

template<class PointType>
void VTPC<PointType>::RadiusFilter(const float &radius){
  pcl::RadiusOutlierRemoval<PointType> outrem;

  outrem.setInputCloud(cloudPtr_);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(1);
  outrem.setNegative(false);
  outrem.setKeepOrganized(false);

  outrem.filter(*cloudPtr_);
}

template<class PointType>
void VTPC<PointType>::viewPointCloud(){

  //pcl::visualization::PCLVisualizer::Ptr viewer_(new pcl::visualization::PCLVisualizer ("3D Viewer"));

  if(viewer_ == NULL){
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    

    viewer_->setBackgroundColor (0, 0, 0);
    
    viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters (); 
    
    viewer_->setCameraPosition(1.03362, 5.26269, 0.231358, -0.000555349, 0.0371576, 0.999309);
    viewer_->setCameraFieldOfView(0.8574994601);
    viewer_->setCameraClipDistances(0.382015, 8.84942);
    viewer_->setPosition(3425, 294);
    viewer_->setSize( 893, 440); 
  
  }

  pcl::visualization::PointCloudColorHandlerGenericField<PointType> intensity_distribution(cloudPtr_, "intensity");
  

  viewer_->addPointCloud<PointType> (cloudPtr_, intensity_distribution, "sample cloud"); 
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 




  std::vector<cv::Point> insidePoints;

  Mat gridImg = Mat(512, 512, CV_8U, gridImg_);

  

  insidePoints.clear();

  cv::findNonZero(gridImg, insidePoints);


  
  int cubeCnt = 0;

  /*
  for( auto const &point : insidePoints){


    viewer_->addCube 
    (grid_[point.y][point.x].voxel_min(0),grid_[point.y][point.x].voxel_max(0),
    grid_[point.y][point.x].voxel_min(1),grid_[point.y][point.x].voxel_max(1), 
    0, 1, 
      1, 1, 1, "cubee" + to_string(cubeCnt++), 0);  
    

  }
  */
  

  std::vector<pcl::PointXYZ> dockcorners;

  for(auto const &object : objDetVec_){

    auto obj = object.second.second;

    

    if(obj.clase != "undefined"){

    

      viewer_->addSphere(obj.centerPoint, 0.01, 255,0,0,"Sphere" + to_string(cubeCnt++), 0);
      //viewer_->addSphere(obj.pt_max, 0.01, 255,0,0,"Sphere" + to_string(cubeCnt++), 0);
      //viewer_->addSphere(obj.pt_min, 0.01, 255,0,0,"Sphere" + to_string(cubeCnt++), 0);

      pcl::PointXYZ ptText;
      std::stringstream ss;

      ptText.x = obj.voxel_max(0);
      ptText.y = obj.voxel_max(1);
      ptText.z = obj.pt_max.z;
      
      ss<<ObjectClassifier(obj)<<"_"<<object.first;

      if(ObjectClassifier(obj) == "dock"){
        dockcorners = FindDockCorners(object.second.second); 


        for(int i = 0; i < dockcorners.size(); i++){
          viewer_->addSphere(dockcorners[i], 0.05, 255,0,0,"Sphere" + to_string(cubeCnt++), 0);
        }


      }
      
        viewer_->addCube 
        (obj.voxel_min(0),obj.voxel_max(0),
        obj.voxel_min(1),obj.voxel_max(1), 
        obj.pt_min.z, obj.pt_max.z, 
          1, 1, 1, "cube" + to_string(cubeCnt++), 0); 
      

      viewer_->addText3D(ss.str(), ptText ,0.1,1,1,1, "text"+ to_string(cubeCnt++), 0);
    

    }
  }

  
  

  viewer_->spinOnce(100);
  
  viewer_->removeAllPointClouds();
  viewer_->removeAllShapes();
  //viewer_->close();
  
}

template<class PointType> 
void VTPC<PointType>::CreateGrid(const float &gridDim){ 

  
  typename pcl::PointCloud<PointType>:: Ptr cloudProjectionPtr (new pcl::PointCloud<PointType>);

  Eigen::Vector3f voxel_min, voxel_max;
  std::vector<int> point_indices;
  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  double octree_min_x, octree_min_y, octree_min_z, 
            octree_max_x, octree_max_y, octree_max_z;
  int x_grid_index, y_grid_index;
  
  gridObj gridObj;

  //clear gridImg
  memset(gridImg_, 0, 512*512);

  //Copy cloud content to modify
  *cloudProjectionPtr = *cloudPtr_;

  //Project PointCloud into the z axis
  for(auto& pt : *cloudProjectionPtr)
    pt.z = 0.0f;

  //Declaring and initialize octree 
  pcl::octree::OctreePointCloudPointVector<PointType> octree(gridDim);  
  octree.setInputCloud(cloudProjectionPtr);
  octree.defineBoundingBox(-5.5,-5.5,-0.4,5.5,5.5,2);
  octree.addPointsFromInputCloud();

  //Obtaining BoundingBox
  octree.getBoundingBox(octree_min_x, octree_min_y, octree_min_z, 
        octree_max_x, octree_max_y, octree_max_z);


  for(auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it)
  {
      
      
      
      auto leaf = it.getLeafContainer();
      
      //Getting leaf's indices, leaf's Bounds, min and max points 
      point_indices.clear();
      leaf.getPointIndices(point_indices);
      octree.getVoxelBounds(it, voxel_min, voxel_max);
      pcl::getMinMax3D(*cloudPtr_, point_indices, min_point, max_point);

      cout<<"1"<<endl;


      float min_height = 0;
      float max_height = 0.3;
      float min_density = 5;
      float max_density = 0;

        if((point_indices.size() > min_density)  && 
            (abs(min_point(2) - max_point(2)) > min_height) &&
            (abs(min_point(2) - max_point(2)) < max_height)
            
            ){
        
          cout<<"2"<<endl;
          //Filling a gridObj Information
          gridObj.centerPoint.x = voxel_min(0) + gridDim/2;
          gridObj.centerPoint.y = voxel_min(1) + gridDim/2;
          gridObj.centerPoint.z = max_point(2) - (abs(max_point(2) - min_point(2))/2);
          gridObj.density = point_indices.size();


          gridObj.pt_min.x = min_point(0);
          gridObj.pt_min.y = min_point(1);
          gridObj.pt_min.z = min_point(2);
          gridObj.pt_max.x = max_point(0);
          gridObj.pt_max.y = max_point(1);
          gridObj.pt_max.z = max_point(2);
          gridObj.voxel_min = voxel_min;
          gridObj.voxel_max = voxel_max;
          gridObj.groupNum = -1;
          gridObj.indices = point_indices;
          gridObj.occupy = true;

          cout<<"3"<<endl;

          //Calculating coordinates in XY plane for indexing gridObj
          x_grid_index = round((gridObj.centerPoint.x+(abs(octree_min_x)-(gridDim/2.0)))/gridDim);
          y_grid_index = round((gridObj.centerPoint.y+(abs(octree_min_y)-(gridDim/2.0)))/gridDim);


          //Assigning gridObj to its corresponding coordinate in the coarseGrid and gridImg


          grid_[x_grid_index][y_grid_index] = gridObj;
          gridImg_[x_grid_index][y_grid_index] = 255;
          

          

          cout<<"5"<<endl;

        }
  }
  
}

template<class PointType> 
void VTPC<PointType>::ClusterGrid(const bool &tracking){

  Mat gridImg = Mat(512, 512, CV_8U, gridImg_);
  Mat prevGridImg = Mat(512, 512, CV_8U, prevGridImg_);
  Mat contour_mask = Mat::zeros( gridImg.size(), CV_8U );
  Mat contour_mask_mod = Mat::zeros( gridImg.size(), CV_8U );

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  
  std::vector<cv::Point> non_zero_contour;
  std::vector<cv::Point> non_zero_tracking;

  int group_number;
  float curr_time;

  //If not tracking reset objects detected vector
  if(!tracking){
    objDetVec_.clear(); 
    groupNumCnt_ = 0;  
  }

  curr_time = float(cloudPtr_->header.stamp / 1000000.0);
  findContours( gridImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

  for( size_t i = 0; i< contours.size(); i++ )
  {

      non_zero_contour.clear();
      non_zero_tracking.clear();
      contour_mask = Mat::zeros( gridImg.size(), CV_8U );

      drawContours( contour_mask, contours, i, cv::Scalar(255,255,255), cv::FILLED);

      //If tracking enable and is not the first iteration
      //The current contour will be dilatated and will be compared with the previous image
      //If there is an overlap it means the current object is close to an object in the previous frame
      //the group number will be passed to the group new image
      //If not a new group number will be assign
      if((tracking) & (isFirstIteration_)){

        dilate(contour_mask, contour_mask_mod, Mat(), Point(-1, -1), 2, 1, 1);      
        bitwise_and(contour_mask_mod, prevGridImg, contour_mask_mod);
        findNonZero(contour_mask_mod, non_zero_tracking);
        
        if(non_zero_tracking.size() > 0){
          group_number = prevGrid_[non_zero_tracking[0].y][non_zero_tracking[0].x].groupNum;
        }
        else{
          group_number = groupNumCnt_++;
        }
      }else{
        group_number = groupNumCnt_++;
      }
      
      //Look for the coordinates in the contour and group the gridObjs
      cv::findNonZero(contour_mask, non_zero_contour);
      
      for( auto const &point : non_zero_contour){

        grid_[point.y][point.x].groupNum = group_number;
      
        if(objDetVec_.find(group_number) == objDetVec_.end()){

          objDetVec_[group_number].second = grid_[point.y][point.x];
          objDetVec_[group_number].first = curr_time;
          
        }else{

          if(tracking && (objDetVec_[group_number].first != curr_time)){
            objDetVec_[group_number].second = grid_[point.y][point.x];
            objDetVec_[group_number].first = curr_time;
          }else{
            objDetVec_[group_number].second = objDetVec_[group_number].second + grid_[point.y][point.x];
          }
          
        }
        
        //Visual Pourposes
        for( auto const &idx: grid_[point.y][point.x].indices){
          cloudPtr_->points[idx].intensity = grid_[point.y][point.x].groupNum;
        }
        

      }
      
  }

  memcpy(prevGrid_, grid_, sizeof(grid_));
  memcpy(prevGridImg_, gridImg_, sizeof(gridImg_));
  isFirstIteration_ = true;

  if(tracking){
    eraseDisapearingObjects(1);
  }

  
}

template<class PointType> 
void VTPC<PointType>::ObjectDetectionPublish(const ros::Publisher &objDetPub){

  usv_perception::obj_detected_list objDetList;
  objDetList.len = objDetVec_.size();

  cout<<objDetVec_.size()<<endl;

  float minR, maxR;

  for( auto &obj : objDetVec_){

    cout<<obj.first<<"-----------------"<<endl;
    
    objDetVec_[obj.first].second.display();

    usv_perception::obj_detected objDet;

    minR = 0;
    maxR = 0;

    minR = pow(obj.second.second.pt_min.x - obj.second.second.centerPoint.x, 2);
    minR += pow(obj.second.second.pt_min.y - obj.second.second.centerPoint.y, 2);
    minR = pow(minR, 0.5);

    maxR = pow(obj.second.second.pt_max.x - obj.second.second.centerPoint.x, 2);
    maxR += pow(obj.second.second.pt_max.y - obj.second.second.centerPoint.y, 2);
    maxR = pow(maxR, 0.5);

    objDet.R = max(minR, maxR);;
    objDet.X = obj.second.second.centerPoint.x;
    objDet.Y = obj.second.second.centerPoint.y;
    objDet.clase = ObjectClassifier(obj.second.second);
    obj.second.second.clase = objDet.clase;
    objDet.id = obj.second.second.groupNum;
  
    if(objDet.clase != "undefined")
      objDetList.objects.push_back(objDet);
    

  }

  objDetPub.publish(objDetList);

}

template<class PointType> 
void VTPC<PointType>::eraseDisapearingObjects(const float &seconds){

  for (auto it = objDetVec_.cbegin(); it != objDetVec_.cend() ; )
  {
    if (it->second.first < float((cloudPtr_->header.stamp / 1000000.0) - seconds)){
      it = objDetVec_.erase(it++);   
    }
    else{
      ++it;
    }
  }
}

template<class PointType> 
string VTPC<PointType>::ObjectClassifier(const gridObj &obj){
  std::stringstream ss;
  ss.str("");
  if((abs(obj.pt_max.z-obj.pt_min.z) < 0.4) &&
      (abs(obj.pt_min.x - obj.pt_max.x) < 0.4) &&
        (abs(obj.pt_min.y - obj.pt_max.y) < 0.4) &&
      (abs(obj.pt_min.x - obj.pt_max.x) > 0) &&
        (abs(obj.pt_min.y - obj.pt_max.y) > 0)
  ){
    ss <<"buoy";
  }

  /*else if(
    (abs(obj.voxel_max(0) - obj.voxel_min(0)) > 1 ) ||
    (abs(obj.voxel_max(1) - obj.voxel_min(1)) > 1 )
  
    ss<<"marker";
  }*/
  
  else{
    ss<<"undefined";
  }

  return ss.str();
}

template<class PointType> 
void VTPC<PointType>::setDockService(const ros::ServiceClient &dockClientServer){
  dock_service_ = dockClientServer;
}

template<class PointType> 
std::vector<pcl::PointXYZ> VTPC<PointType>::FindDockCorners(const gridObj &obj){
  
  pcl::ExtractIndices<PointType> extract;
  typename pcl::PointCloud<PointType>::Ptr cloud_p(new pcl::PointCloud<PointType>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers->indices = obj.indices;
  
  extract.setInputCloud(cloudPtr_);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_p);

  Mat img = Mat::zeros(Size(500, 500) , CV_8U );
  Mat img2;
  cvtColor( img,img2, CV_GRAY2RGB);

  usv_perception::dock_corners srv;
  std::vector<geometry_msgs::Point32> points;

  for(auto& pt : *cloud_p){


    geometry_msgs::Point32 point;

    point.x = pt.x - obj.centerPoint.x;
    point.y = pt.y - obj.centerPoint.y;
    point.z = pt.z;

    points.push_back(point);

    pt.x = ceil((pt.x - obj.centerPoint.x)*100)+ 250;
    pt.y = ceil((pt.y - obj.centerPoint.y)*100)+ 250;

    img.at<Vec3b>(pt.y,pt.x) = 255;

  }

  geometry_msgs::Point32 pointCenter;

  pointCenter.x =obj.centerPoint.x;
  pointCenter.y= obj.centerPoint.y;

  srv.request.pointCoordinates = points;
  srv.request.centerPoint = pointCenter;

  dock_service_.call(srv);

  std::vector<pcl::PointXYZ> dockcorners;
  for(auto &pt: srv.response.dockCoordinates){

    //pt.x += obj.centerPoint.x;
    //pt.y += obj.centerPoint.y;
    pt.z = obj.pt_max.z;

    pcl::PointXYZ pt2(pt.x, pt.y, pt.z);

    cout<<pt2<<endl;

    dockcorners.push_back(pt2);

  }

  //cout<<"Response "<<srv.response.dockCoordinates.size()<<endl;

  
  
  return dockcorners;
  
  
}

template<class PointType> 
void VTPC<PointType>::addNoise(const int &n){


  

  for(int i = 0; i < n; i++){

    pcl::PointXYZI p;


    p.x = (float) rand()/RAND_MAX*10 - 5;
    p.y = (float) rand()/RAND_MAX*10 - 5;
    p.z = 0;

    cloudPtr_->points.push_back(p);
  }
  

}


//template class VTPC<pcl::PointXYZ>;
template class VTPC<pcl::PointXYZI>;
//template class VTPC<pcl::PointXYZRGB>;

