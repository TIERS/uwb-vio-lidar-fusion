#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"


class PCL_conv
{
	private:
		ros::NodeHandle n;
		
		ros::Publisher utuTIERS_drone_cloud_pub ;
		ros::Publisher deca_drone_cloud_pub ;
    ros::Publisher aux_cloud_pub;
    ros::Publisher meanpoint_pub;

    ros::Subscriber cloud_sub ;
		ros::Subscriber utuTIERS_pos_sub ;
    ros::Subscriber deca_pos_sub ;
		ros::Subscriber tfmini_sub ;

    float z_pos_drone ;
		geometry_msgs::Pose drone_pose;
    geometry_msgs::Point meanPoint;
    
    bool received_utuTIERS, received_deca = false;
 

	public:
		PCL_conv();
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
		void utuTIERS_pos_cb( geometry_msgs::Pose pose);
    void deca_pos_cb( geometry_msgs::Pose pose);
    void tfmini_cb(sensor_msgs::Range msg);
		void kdtree_search();
};

//***********CONSTRUCTOR*******//
PCL_conv::PCL_conv(void){
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_sub = n.subscribe("/lslidar_point_cloud", 1000, &PCL_conv::cloud_cb, this);
	utuTIERS_pos_sub = n.subscribe("/utuTIERS/tag/position/mean", 1000, &PCL_conv::utuTIERS_pos_cb, this);
  deca_pos_sub = n.subscribe("/dwm1001/tag/drone/position", 1000, &PCL_conv::deca_pos_cb, this);
	tfmini_sub = n.subscribe("/tfmini_ros_node/TFmini", 1000, &PCL_conv::tfmini_cb, this);

	utuTIERS_drone_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("utuTIERS_drone_points", 1000);
  deca_drone_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("deca_drone_points", 1000);
  aux_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("aux_drone_points", 1000);
  meanpoint_pub = n.advertise<geometry_msgs::Point> ("lidar_mean_point", 1000);
}

/*CALLBACK FUNCTION TO GET DRONE POSITION*/
void PCL_conv::utuTIERS_pos_cb(geometry_msgs::Pose pose){
	drone_pose = pose;
  drone_pose.position.z = z_pos_drone - 0.8; //substract the z position of the reference (3d lidar) to the lidar1D measurement 
  received_utuTIERS = true;
}
void PCL_conv::deca_pos_cb(geometry_msgs::Pose pose){
	drone_pose = pose;
  drone_pose.position.z = z_pos_drone - 0.8; //substract the z position of the reference (3d lidar) to the lidar1D measurement 
  received_deca = true;
}


void PCL_conv::tfmini_cb(sensor_msgs::Range msg){
	z_pos_drone = msg.range;
}


/*CALLBACK TO CONVERT LIDAR DATA TO POINT CLOUD*/
void PCL_conv::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){ //convert msgs pointcloud2 to pcl cloud

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

	kdtree_search();
}

 
 //****create a kdtree and look for the nearest neighbors and clusters****//
void PCL_conv::kdtree_search()
{

	 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;
  
  meanPoint.x = 0.0;
  meanPoint.y = 0.0;
  meanPoint.z = 0.0;

  searchPoint.x = drone_pose.position.x;//1.23f;
  searchPoint.y = drone_pose.position.y;//5.85f;
  searchPoint.z = drone_pose.position.z;//-0.52f;

  // K nearest neighbor search

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

 searchPoint.x = (*cloud)[ pointIdxNKNSearch[0] ].x  ;
 searchPoint.y = (*cloud)[ pointIdxNKNSearch[0] ].y  ;
 searchPoint.z = (*cloud)[ pointIdxNKNSearch[0] ].z  ;
  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 0.3f;

  
  // create point cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr droneCloud (new pcl::PointCloud<pcl::PointXYZ>);
  droneCloud->header.frame_id = "laser_link";


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
     
      pcl::PointXYZ newPoint;
      newPoint.x = (*cloud)[ pointIdxRadiusSearch[i] ].x ;
      newPoint.y = (*cloud)[ pointIdxRadiusSearch[i] ].y ;
      newPoint.z = (*cloud)[ pointIdxRadiusSearch[i] ].z ;
      droneCloud->points.push_back(newPoint);

      meanPoint.x += newPoint.x;
      meanPoint.y += newPoint.y;
      meanPoint.z += newPoint.z;

        
    }

    //calculate mean point to take it as the lidar's estimation
    meanPoint.x = meanPoint.x/pointIdxRadiusSearch.size();
    meanPoint.y = meanPoint.y/pointIdxRadiusSearch.size();
    meanPoint.z = meanPoint.z/pointIdxRadiusSearch.size();

    
    
    // publish point cloud
    if(droneCloud->size() > 0 ) {
      ros::Time time_st = ros::Time::now ();

      droneCloud->header.stamp= time_st.toNSec()/1e3;

      auxCloud->header.stamp= time_st.toNSec()/1e3;
      aux_cloud_pub.publish (auxCloud);
      
      if(received_utuTIERS){
        utuTIERS_drone_cloud_pub.publish (droneCloud);
        received_utuTIERS = false;
      }
        
      else if(received_deca){
        deca_drone_cloud_pub.publish (droneCloud);
        received_deca = false;
      }
      meanpoint_pub.publish(meanPoint);
    }
  
  }

}

int main(int argc, char **argv)
{
	// *
	// * The ros::init() function needs to see argc and argv so that it can perform
	// * any ROS arguments and name remapping that were provided at the command line.
	// * For programmatic remappings you can use a different version of init() which takes
	// * remappings directly, but for most command-line programs, passing argc and argv is
	// * the easiest way to do it.  The third argument to init() is the name of the node.
	// *
	// * You must call one of the versions of ros::init() before using any other
	// * part of the ROS system.
	

	

	ros::init(argc, argv, "pcl_conver");

	PCL_conv pcl_conver;
	
	ros::spin();

	return 0;
}
