#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include "kf_tracker/featureDetection.h"
#include "kf_tracker/kalman.hpp"
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
 
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>
#include "turtlebot3_detect/GetObst.h"

using namespace std;


ros::Publisher objID_pub;



  std::vector<KalmanFilter> KFs;
  ros::Publisher markerPub;

  Eigen::Vector6d state; // [x,y,v_x,v_y,w,h]
  Eigen::Vector4d measurement; // [z_x,z_y,z_w,z_h] 

  std::vector<int> objID;// Output of the data association using KF
 // measurement.setTo(Scalar(0));
  float cluster_size=1;

bool firstFrame=true;

// calculate euclidean distance of two points
  double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }
/*
//Count unique object IDs. just to make sure same ID has not been assigned to two KF_Trackers.  
int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); // O(n) where n = distance(v.end(), v.begin())
    sort(v.begin(), v.end()); // Average case O(n log n), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then sort_heap.

    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique section of the range
    auto unique_end = unique(v.begin(), v.end()); // Again n comparisons
    return distance(unique_end, v.begin()); // Constant time for random access iterators (like vector's)
}
*/

/*

objID: vector containing the IDs of the clusters that should be associated with each KF_Tracker
objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

std::pair<int,int> findIndexOfMin(std::vector<std::vector<float> > distMat)
{
   // cout<<"findIndexOfMin cALLED\n";
    std::pair<int,int>minIndex;
    float minEl=std::numeric_limits<float>::max();
   // cout<<"minEl="<<minEl<<"\n";
    for (int i=0; i<distMat.size();i++)
        for(int j=0;j<distMat.at(0).size();j++)
        {
            if( distMat[i][j]<minEl)
            {
                minEl=distMat[i][j];
                minIndex=std::make_pair(i,j);

            }

        }
   // cout<<"minIndex="<<minIndex.first<<","<<minIndex.second<<"\n";
    return minIndex;
}
void KFT(const std_msgs::Float32MultiArray ccs)
{



// First predict, to update the internal statePre variable
  std::vector<Eigen::VectorXd> pred;
  for (int i =0; i < KFs.size(); i++)
  {
    pred.push_back(KFs.at(i).state());
  }
 
 
    // Get measurements
    // Extract the position of the clusters forom the multiArray. To check if the data
    // coming in, check the .z (every third) coordinate and that will be 0.0
    std::vector<geometry_msgs::Point> clusterCenters;//clusterCenters
   
    int i=0;
    for (std::vector<float>::const_iterator it=ccs.data.begin();it!=ccs.data.end();it+=3)
    {
        geometry_msgs::Point pt;
        pt.x=*it;
        pt.y=*(it+1);
        pt.z=*(it+2);

        clusterCenters.push_back(pt);
       
    }

    std::vector<geometry_msgs::Point> KFpredictions;
    i=0;
    for (auto it=pred.begin();it!=pred.end();it++)
    {
        geometry_msgs::Point pt;
        pt.x=(*it)(0);
        pt.y=(*it)(1);
        pt.z=0;
        KFpredictions.push_back(pt);
        
    }
  // cout<<"Got predictions"<<"\n";

    
    
    // Find the cluster that is more probable to be belonging to a given KF.
    objID.clear();//Clear the objID vector
    objID.resize(KFs.size());//Allocate default elements so that [i] doesnt segfault. Should be done better
    // Copy clusterCentres for modifying it and preventing multiple assignments of the same ID
    std::vector<geometry_msgs::Point> copyOfClusterCenters(clusterCenters);
    std::vector<std::vector<float> > distMat;

    for(int filterN=0;filterN<KFpredictions.size();filterN++)
    {
        std::vector<float> distVec;
        for(int n=0;n<clusterCenters.size();n++)
        {
            distVec.push_back(euclidean_distance(KFpredictions[filterN],copyOfClusterCenters[n]));
        }

        distMat.push_back(distVec);
      
    }




    for(int clusterCount=0;clusterCount<KFs.size();clusterCount++)
    {
        // 1. Find min(distMax)==> (i,j);
        std::pair<int,int> minIndex(findIndexOfMin(distMat));
     //    cout<<"Received minIndex="<<minIndex.first<<","<<minIndex.second<<"\n";
        // 2. objID[i]=clusterCenters[j]; counter++
        objID[minIndex.first]=minIndex.second;
    
        // 3. distMat[i,:]=10000; distMat[:,j]=10000
        distMat[minIndex.first]=std::vector<float>(KFs.size(),10000.0);// Set the row to a high number.
        for(int row=0;row<distMat.size();row++)//set the column to a high number
        {
            distMat[row][minIndex.second]=10000.0;
        }
        // 4. if(counter<6) got to 1.
     //   cout<<"clusterCount="<<clusterCount<<"\n";

    }

   // cout<<"Got object IDs"<<"\n";
    //countIDs(objID);// for verif/corner cases

    //display objIDs
  /* DEBUG
    cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        cout<<*it<<" ,";
    cout<<"\n";
    */

    visualization_msgs::MarkerArray clusterMarkers;

     for (int i=0;i<KFs.size();i++)
     {
        visualization_msgs::Marker m;

        m.id=i;
        m.type=visualization_msgs::Marker::CUBE;
        m.header.frame_id="base_link";
        m.scale.z=0.3;
        m.action=visualization_msgs::Marker::ADD;
        m.color.a=1.0;
        m.color.r=i%2?1:0;
        m.color.g=i%3?1:0;
        m.color.b=i%4?1:0;

       //geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
       m.pose.position.x=pred[i][0];
       m.pose.position.y=pred[i][1];
       m.pose.position.z=0;

       m.scale.x=pred[i][4];
       m.scale.y=pred[i][5];
       
       visualization_msgs
       clusterMarkers.markers.push_back(m);
     }

   // prevClusterCenters=clusterCenters;

     markerPub.publish(clusterMarkers);




    std_msgs::Int32MultiArray obj_id;
    for(auto it=objID.begin();it!=objID.end();it++)
        obj_id.data.push_back(*it);
    // Publish the object IDs
    objID_pub.publish(obj_id);
    // convert clusterCenters from geometry_msgs::Point to floats
    for (int i=0;i<KFs.size();i++)
    {
        Eigen::Vector6d measMat(clusterCenters[objID[i]].x,  clusterCenters[objID[i]].y)
        if (!(measMat.at[0,0]==0.0f || measMat[1,0]==0.0f))
            KFs.at(i).update(measMat);
    }

 
   
}
void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = "base_link";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);

}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
   // cout<<"IF firstFrame="<<firstFrame<<"\n";
    // If this is the first frame, initialize kalman filters for the clustered objects
if (firstFrame)
{   
// Process the point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);
  if(input_cloud->empty())
  {
    return;
  }

  tree->setInputCloud (input_cloud);


  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); 
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);

  
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;
    // Cluster centroids
  std::vector<pcl::PointXYZ> clusterCentroids;
  std::vector<pcl::PointXYZ> clustersize;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
          
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
         float x=0.0; float y=0.0;
         int numPts=0;
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
          {
          
                  cloud_cluster->points.push_back(input_cloud->points[*pit]);
                  x+=input_cloud->points[*pit].x;
                  y+=input_cloud->points[*pit].y;
                  numPts++;


                  //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
                  //                                          origin);
                  //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

         
      pcl::PointXYZ centroid;
      centroid.x=x/numPts;
      centroid.y=y/numPts;
      centroid.z=0.0;
      
      cluster_vec.push_back(cloud_cluster);

      //Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
      clustersize.push_back(size);

    }

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < KFs.size()){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size()<KFs.size())
    {
      pcl::PointXYZ centroid;
      centroid.x=0.0;
      centroid.y=0.0;
      centroid.z=0.0;
      
       clusterCentroids.push_back(centroid);
    }
    
      for(int i =0; i< KFs.size(); i++)
      {
       // Set initial state
        Eigen.Vector6d temp(clusterCentroids.at(i).x, clusterCentroids.at(i).y, 0,0, clustersize.at(i).x, clustersize.at(i).y )
       KFs.at(i).init(temp);
     }
     
     firstFrame=false;

   
}

 
else
{ 
  //cout<<"ELSE firstFrame="<<firstFrame<<"\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);
  if(input_cloud->empty())
  {
    return;
  }
  /* Here we are creating a vector of PointIndices, which contains the actual index
   * information in a vector<int>. The indices of each detected cluster are saved here.
   * Cluster_indices is a vector containing one instance of PointIndices for each detected 
   * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
   */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
//cout<<"PCL init successfull\n";
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);
//cout<<"PCL extract successfull\n";
  /* To separate each cluster out of the vector<PointIndices> we have to 
   * iterate through cluster_indices, create a new PointCloud for each 
   * entry and write all points of the current cluster in the PointCloud. 
   */
  //pcl::PointXYZ origin (0,0,0);
  //float mindist_this_cluster = 1000;
  //float dist_this_point = 1000;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

     // Cluster centroids
  std::vector<pcl::PointXYZ> clusterCentroids;

  if (cluster_indices.size()> (int)cluster_size)
    cluster_size+=0.1;
  else if (cluster_indices.size()<(int)cluster_size)
    cluster_size-=0.1;
  


  ROS_DEBUG_STREAM("clusters"<< cluster_indices.size()<<"  smoothed"<< cluster_size <<" KFS"<< KFs.size()<<endl);

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
   {
        float x=0.0; float y=0.0;
         int numPts=0;
       //  mindist_this_cluster=
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
          {
          
                  cloud_cluster->points.push_back(input_cloud->points[*pit]);


                  x+=input_cloud->points[*pit].x;
                  y+=input_cloud->points[*pit].y;
                  numPts++;

               //   dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],origin);
                //  mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

    pcl::PointXYZ centroid;
      centroid.x=x/numPts;
      centroid.y=y/numPts;
      centroid.z=0.0;
      
      cluster_vec.push_back(cloud_cluster);

      //Get the centroid of the cluster
      clusterCentroids.push_back(centroid);

    }

  if((int)cluster_size>KFs.size())
    { 
      cout<<"increasing size";
      KFs.push_back(cv::KalmanFilter(stateDim,measDim,ctrlDim,CV_32F));
      firstrun(KFs.size()-1);

       KFs.at(KFs.size()-1).statePre.at<float>(0)=clusterCentroids.at(KFs.size()-1).x;
       KFs.at(KFs.size()-1).statePre.at<float>(1)=clusterCentroids.at(KFs.size()-1).y;
       KFs.at(KFs.size()-1).statePre.at<float>(2)=0;// initial v_x
       KFs.at(KFs.size()-1).statePre.at<float>(3)=0;//initial v_y
      cout<<"done increasing"<<endl;

    }
   else if ((int)cluster_size<KFs.size())
   {
     KFs.pop_back();
   } 

   // cout<<"cluster_vec got some clusters\n";

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < KFs.size()){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }

     while (clusterCentroids.size()<KFs.size())
    {
      pcl::PointXYZ centroid;
      centroid.x=0.0;
      centroid.y=0.0;
      centroid.z=0.0;
      
       clusterCentroids.push_back(centroid);
    }


    std_msgs::Float32MultiArray cc;
    for(int i=0;i<KFs.size();i++)
    {
        cc.data.push_back(clusterCentroids.at(i).x);
        cc.data.push_back(clusterCentroids.at(i).y);
        cc.data.push_back(clusterCentroids.at(i).z);    

    }
   // cout<<"6 clusters initialized\n";

    //cc_pos.publish(cc);// Publish cluster mid-points.
   // cout<<"a";
    KFT(cc);
  // cout<<"b"<<endl;
    // int i=0;
    // bool publishedCluster[KFs.size()];
    // for(auto it=objID.begin();it!=objID.end();it++)
    //     { //cout<<"Inside the for loop\n";
    //         cout<<"objID"<<i<<endl;
    //     //    if(pub_clusters.size()>i)
    //         //  publish_cloud(pub_clusters.at(i) ,cluster_vec[*it]);
    //         publishedCluster[i]=true;
    //         i++;
        
    //     }

} 

}   


 bool getobstacles(turtlebot3_detect::GetObst::Request  &req,
         turtlebot3_detect::GetObst::Response &res)
{
    geometry_msgs::Point pts[KFs.size()];
    res.points.clear();
    for (int i =0; i < KFs.size(); i++)
     {
        cv::Mat pred = KFs.at(i).predict();
        geometry_msgs::Point32 p;
        p.x=pred.at<float>(0);
        p.y=pred.at<float>(1);
        p.z=pred.at<float>(2);
        res.points.push_back(p);
        
    }
    res.head.stamp= ros::Time::now();
    return true;
} 

int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"kf_tracker");
    ros::NodeHandle nh;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
   
    // Publishers to publish the state of the objects (pos and vel)
    //objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);



//cout<<"About to setup callback\n";

// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("filtered_cloud", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  int count;
  nh.getParam("object_limit", count);

  for( int i =0; i<count; i++)
    {
      cv::KalmanFilter temp(stateDim,measDim,ctrlDim,CV_32F);
      KFs.push_back(temp);
   //   pub_clusters.push_back( nh.advertise<sensor_msgs::PointCloud2> ("cluster_"+to_string(i), 1, true)); 

  }

    //ros::Subscriber c1=nh.subscribe("ccs",100,KFT); 
    objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
/* Point cloud clustering
*/
    
  //cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
  markerPub= nh.advertise<visualization_msgs::MarkerArray> ("viz",1);
  ros::ServiceServer obstacles_service = nh.advertiseService("get_obstacles_data", getobstacles);

/* Point cloud clustering
*/    
  ros::spin();
}
