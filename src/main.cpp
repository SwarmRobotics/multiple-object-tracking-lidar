#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
// #include "kf_tracker/featureDetection.h"
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
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
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
#include <cmath> 
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>
#include "turtlebot3_detect/GetObst.h"
#include <vector>
using namespace std;

ros::Publisher objID_pub;
 string frame_id;

  int state_size=8; //[x,y,r1,r2,quart[4]]
  int meas_size=8; //[x,y,r1,r2,quart[4]]
    
  std::vector<KalmanFilter> KFs;
  ros::Publisher markerPub;


  std::vector<int> objID;// Output of the data association using KF
 // measurement.setTo(Scalar(0));
  float cluster_size=1;

bool firstFrame=true;

// calculate euclidean distance of two points
  double euclidean_distance(const Eigen::Vector8d& p1, const Eigen::Vector4d& p2)
  {
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1])) ;
  }
  double euclidean_distance(double a,double b, double c, double d )
  {
    return sqrt((a - c) * (a - c) + (b - d) * (b - d)) ;
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

std::pair<int,int> findIndexOfMin(std::vector<std::vector<float> >& distMat)
{
   // ROS_DEBUG_STREAM("findIndexOfMin cALLED\n";
    std::pair<int,int>minIndex;
    float minEl=std::numeric_limits<float>::max();
   // ROS_DEBUG_STREAM("minEl="<<minEl<<"\n";
    for (int i=0; i<distMat.size();i++)
        for(int j=0;j<distMat.at(0).size();j++)
        {
            if( distMat[i][j]<minEl)
            {
                minEl=distMat[i][j];
                minIndex=std::make_pair(i,j);

            }

        }
   // ROS_DEBUG_STREAM("minIndex="<<minIndex.first<<","<<minIndex.second<<"\n";
    return minIndex;
}
void KFT(const std::vector<Eigen::Vector4f>& centers, const std::vector<Eigen::Vector2f>& radii, const std::vector<Eigen::Quaternionf>& yaws )
{



// First predict, to update the internal statePre variable
  std::vector<Eigen::Vector8d> pred;
  pred.resize(KFs.size());
     visualization_msgs::MarkerArray clusterMarkers;

  for (int i =0; i < KFs.size(); i++)
  {
       pred[i]=KFs.at(i).state();  







       // assert (pred[i].rows()==2);
        visualization_msgs::Marker m;

        m.id=i;
        m.type=visualization_msgs::Marker::CUBE;
        m.header.frame_id=frame_id;
        m.action=visualization_msgs::Marker::ADD;
        m.color.a=1.0;
        m.color.r=i%2?1:0;
        m.color.g=i%3?1:0;
        m.color.b=i%4?1:0;

       //geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
       m.pose.position.x=pred.at(i)[0];
       m.pose.position.y=pred.at(i)[1];
       m.pose.position.z=0;

      // double yaw=pred.at(i)[4];
      // Eigen::Matrix3f matYaw(3, 3);
      //  matYaw << cos(yaw), sin(yaw), 0.0f,
      //   -sin(yaw), cos(yaw), 0.0f,  //z
      //   0.0f, 0.0f, 1.0f;
       
       Eigen::Quaternionf quatFromRot(pred.at(i)[7], pred.at(i)[4],pred.at(i)[5],pred.at(i)[6]);
        quatFromRot.normalize();
       m.pose.orientation.x=quatFromRot.x();
       m.pose.orientation.y=quatFromRot.y();
       m.pose.orientation.z=quatFromRot.z();
       m.pose.orientation.w=quatFromRot.w();


       m.scale.x=pred.at(i)[2];
       m.scale.y=pred.at(i)[3];
       
        m.scale.z=0.3;//pred[i][2];
      
     //  cout<<"state"<<KFs.at(i).state()<<endl;
       clusterMarkers.markers.push_back(m);
    }
    //  ROS_DEBUG_STREAM("preds and marker  made ");


     markerPub.publish(clusterMarkers);




    
    // Find the cluster that is more probable to be belonging to a given KF.
    objID.clear();//Clear the objID vector
    objID.resize(KFs.size());//Allocate default elements so that [i] doesnt segfault. Should be done better
    std::vector<std::vector<float> > distMat;

    //ROS_DEBUG_STREAM("clear objID");

    for(int filterN=0;filterN<pred.size();filterN++)
    {
        std::vector<float> distVec;
        for(int n=0;n<centers.size();n++)
        {   
            double d=euclidean_distance( pred[filterN](0),pred[filterN](1), centers.at(n)(0),centers.at(n)(1) );
            distVec.push_back(d);
         //   cout<<d<<" ";
        }
       // cout<<endl;
        distMat.push_back(distVec);
      
    }


   // ROS_DEBUG_STREAM("assign distance Matrix");





    for(int clusterCount=0;clusterCount<KFs.size();clusterCount++)
    {
        // 1. Find min(distMax)==> (i,j);
        std::pair<int,int> minIndex(findIndexOfMin(distMat)); // filterID, dataID
     //    ROS_DEBUG_STREAM("Received minIndex="<<minIndex.first<<","<<minIndex.second<<"\n";
        // 2. objID[i]=clusterCenters[j]; counter++
        objID[minIndex.first]=minIndex.second;
    
        // 3. distMat[i,:]=10000; distMat[:,j]=10000
        for(int col=0; col<KFs.size();col++)
        {
          distMat[minIndex.first][col]=std::numeric_limits<float>::max();
        }
        for(int row=0;row<distMat.size();row++)//set the column to a high number
        {
            distMat[row][minIndex.second]=std::numeric_limits<float>::max();
        }
        // 4. if(counter<6) got to 1.
     //   ROS_DEBUG_STREAM("clusterCount="<<clusterCount<<"\n";

    }
    // for (int i=0; i<  objID.size(); i++)
    // {
    //   cout<<objID[i]<<' ';
    // }

 //   cout<<endl;

   // ROS_DEBUG_STREAM("assign objectID Matrix");

   // ROS_DEBUG_STREAM("Got object IDs"<<"\n";
    //countIDs(objID);// for verif/corner cases

    //display objIDs
  /* DEBUG
    ROS_DEBUG_STREAM("objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        ROS_DEBUG_STREAM(*it<<" ,";
    ROS_DEBUG_STREAM("\n";
    */

  


    std_msgs::Int32MultiArray obj_id;
    for(auto it=objID.begin();it!=objID.end();it++)
        obj_id.data.push_back(*it);
    // Publish the object IDs
    objID_pub.publish(obj_id);
   // ROS_DEBUG_STREAM("published objectID Matrix");


    for (int i=0;i<KFs.size();i++)
    {
      // ROS_DEBUG_STREAM("updating");
         Eigen::Vector8d measMat;
          measMat<< centers[objID[i]](0,0),  centers[objID[i]](1,0), radii[objID[i]](0),radii[objID[i]](1), yaws[objID[i]].x(),yaws[objID[i]].y(),yaws[objID[i]].z() ,yaws[objID[i]].w(); //[x,y,r1,r2]
      //    cout<<i<<' '<<objID[i]<<' '<<measMat<<endl;
          KFs.at(i).update(measMat);

      // ROS_DEBUG_STREAM("updating done");
    }

 
     //  ROS_DEBUG_STREAM("done kd");

}



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
//  ROS_DEBUG_STREAM("start");
   // ROS_DEBUG_STREAM("IF firstFrame="<<firstFrame<<"\n";
    // If this is the first frame, initialize kalman filters for the clustered objects

  std::vector<Eigen::Vector4f> clusterCentroids;
  std::vector<Eigen::Vector2f> radii;
  std::vector<Eigen::Quaternionf> quats;
  
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

//ROS_DEBUG_STREAM("culster start");
  tree->setInputCloud (input_cloud);


  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); 
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (180);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);
//ROS_DEBUG_STREAM("culster made");
  
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
    // Cluster centroids


  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
  {
          
         
      Eigen::Vector4f centroid;
      if (pcl::compute3DCentroid(*input_cloud, *it, centroid)>0)
      {
        //Get the centroid of the cluster
        clusterCentroids.push_back(centroid);

        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*input_cloud, *it, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
           
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input_cloud, *it, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*input_cloud, *it, minPoint, maxPoint);
       // const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
       quats.push_back(bboxQuaternion);
       // const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + centroid.head<3>();
        // pcl::visualization::PCLVisualizer *visu;
        // visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
        // int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
        // visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
        // visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
        // visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
        // visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
        // visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
        // visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
        // double width =maxPoint(0)-minPoint(0);
        // double height =maxPoint(0)-minPoint(0);
        // if width
        radii.push_back(Eigen::Vector2f(maxPoint(0)-minPoint(0), maxPoint(1)-minPoint(1)) );


        // const auto x = bboxQuaternion.y();
        // const auto y = bboxQuaternion.z();
        // const auto z = bboxQuaternion.x();
        // const auto w = bboxQuaternion.w();
        // double yaw = atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
        // yaws.push_back(yaw);
      }

                                                                                ///    the signs are different and the box doesn't get correctly oriented in some cases.
    }
   // ROS_DEBUG_STREAM("centroid found");


 

    while (clusterCentroids.size()<KFs.size())
    {
        Eigen::Vector4f centroid= Eigen::Vector4f::Zero();
        Eigen::Vector2f box= Eigen::Vector2f::Zero();
        Eigen::Quaternionf ang(1.0,0.0,0.0,0.0);

        clusterCentroids.push_back(centroid);
        radii.push_back(box);
        quats.push_back(ang);
    }
    

    if (firstFrame)
    { 
          for(int i =0; i< KFs.size(); i++)
          {
           // Set initial state
            Eigen::Vector8d temp;
            temp<< clusterCentroids.at(i)(0,0), clusterCentroids.at(i)(1,0), radii.at(i)(0), radii.at(i)(1), quats.at(i).x(), quats.at(i).y(), quats.at(i).z(), quats.at(i).w();
            KFs.at(i).init(0,temp);
          //    cout<<"init "<<temp<<endl;
           }
         
         firstFrame=false;
       }
    else
    { 
   

      if (cluster_indices.size()> (int)cluster_size)
        cluster_size+=0.1;
      else if (cluster_indices.size()<(int)cluster_size)
        cluster_size-=0.1;
   

        if((int)cluster_size>KFs.size() and cluster_indices.size()>KFs.size())
          { 
          //  ROS_DEBUG_STREAM("increasing size");
            KalmanFilter temp(state_size, meas_size, 0.1);
            Eigen::Vector8d init_value = Eigen::Vector8d::Zero();
            init_value(7)=1.0;
            temp.init(0, init_value);
            
            KFs.push_back(temp);
           // ROS_DEBUG_STREAM("done increasing");
          }
         else if ((int)cluster_size<KFs.size())
         {
             KFs.pop_back();
         } 

         KFT(clusterCentroids, radii, quats);


    } 

}   


 bool getobstacles(turtlebot3_detect::GetObst::Request  &req,
         turtlebot3_detect::GetObst::Response &res)
{
    //ROS_DEBUG_STREAM("obst requested");
    res.points.clear();
    for (int i =0; i < KFs.size(); i++)
     {
        Eigen::Vector8d pred = KFs.at(i).state();

        geometry_msgs::Point32 p;
        p.x=pred[0];
        p.y=pred[1];
        res.points.push_back(p);
        //res.radii.push_back(pred[2]) ;
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



//ROS_DEBUG_STREAM("About to setup callback\n";

// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("filtered_cloud", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  int count;
  nh.getParam("object_limit", count);
  int myID;
  nh.getParam("agent_ID", myID);
  frame_id = "burger_";

  frame_id= frame_id.append(to_string(myID));
  frame_id =frame_id.append("/base_link_");
  frame_id= frame_id.append(to_string(myID));
  for( int i =0; i<count; i++)
    {
      KFs.push_back(KalmanFilter(state_size, meas_size, 0.1));
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
