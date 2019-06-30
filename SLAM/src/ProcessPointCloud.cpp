#include "ProcessPointCloud.h"


ProcessCloud::ProcessCloud()
{
    cloud.reset( new pcl::PointCloud<pcl::PointXYZI> );
}

ProcessCloud::~ProcessCloud()
{

}

void ProcessCloud::setInputCloud( const pcl::PointCloud<pcl::PointXYZI>::Ptr& input )
{
    cloud = input;
}

void ProcessCloud::getCloud( pcl::PointCloud<pcl::PointXYZI>::Ptr& input )
{
    input = cloud;
}


void ProcessCloud::VoxelSampling( pcl::PointCloud<pcl::PointXYZI>::Ptr& result, float leafsize )
{
    pcl::PCLPointCloud2::Ptr input_cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    result->resize(cloud->size());
    std::cout<< " converting pcl to cloud2 " << std::endl;
    pcl::toPCLPointCloud2(*cloud, *input_cloud);
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    std::cout<< " before setting input cloud for voxel " << std::endl;
    sor.setInputCloud ( input_cloud);
    sor.setLeafSize ( leafsize, leafsize, leafsize);
    sor.filter (*cloud_filtered );

    std::cout<< " after filter " << std::endl;
    pcl::fromPCLPointCloud2( *cloud_filtered, *result);
}

void ProcessCloud::filterPointCloud( pcl::PointCloud<pcl::PointXYZI>::Ptr& result, std::string type = "voxel" )
{
    if( type == "voxel" )
        VoxelSampling( result, 0.05f );
}
