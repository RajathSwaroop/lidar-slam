#include <iostream>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>


class Mapper
{
    /*
    */
    
public:
    Mapper();
    ~Mapper();

    pcl::PointCloud<pcl::PointXYZI>::Ptr map;
    void setId( const long long Id );
    long long getId();
    void initGICP();
    void setReferenceCloud( const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void getReferenceCloud( pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud );
    void setQueryCloud( const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud );
    void computePoseDifferenceNDT( );
    void computePoseDifferenceGICP( );
    void computeOdometry();
    void printOdometry();
    void publishMap( sensor_msgs::PointCloud2& output );
    void getOdometry( Eigen::Affine3d& odom );
    void addToMap();
    void run( const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const long long id );

private:
    long long id;
    Eigen::Affine3d pose;
    Eigen::Affine3d odometry;
    pcl::PointCloud<pcl::PointXYZI>::Ptr reference;// (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr query;// (new pcl::PointCloud<pcl::PointXYZI>);
    
    std::vector<Eigen::Affine3d> posehistory;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_data;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr oct_map;
    
};
