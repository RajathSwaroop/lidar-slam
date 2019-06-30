#include <iostream>
#include <string>

#include <fstream>
#include <dirent.h>


#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
#define VTK_RENDERING_BACKEND_OPENGL_VERSION 1
#include <pcl/visualization/cloud_viewer.h>


#include "nav_msgs/Odometry.h"
#include "ProcessPointCloud.h"
#include <Eigen/Geometry>
#include "Mapper.h"

ros::Publisher odom_pub;
ros::Publisher map_pub;
ros::Subscriber points_sub;

Mapper slam;
long long seq = 0;

using namespace std;

#define rad2deg 57.2958;

pcl::visualization::CloudViewer viewer("Cloud Viewer");

void publishOdometry( Eigen::Affine3d& odom )
{
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = odom.translation()[0];
    msg.pose.pose.position.y = odom.translation()[1];
    msg.pose.pose.position.z = odom.translation()[2]; 

    Eigen::Vector3d euler = odom.rotation().eulerAngles(2, 1, 0);

    //euler[0] *= rad2deg;
    //euler[1] *= rad2deg;
    //euler[2] *= rad2deg;

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd( euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd( euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd( euler[2], Eigen::Vector3d::UnitZ());
    
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z(); 
    msg.pose.pose.orientation.w = q.w();
    
    msg.header.frame_id = "odom";
    msg.header.stamp = ros::Time::now();
    msg.child_frame_id = "base_footprint";

    odom_pub.publish( msg );
}

void displayPointCloud( const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    viewer.showCloud( cloud );
}

/*
void LidarCallBack( const sensor_msgs::PointCloud2ConstPtr& points )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*points, *cloud);

    std::cout<< " read sensor msg with seq " << points->header.seq << std::endl;
    slam.run( cloud, seq );

    Eigen::Affine3d odom;
    slam.getOdometry( odom );

    publishOdometry( odom );
    seq++;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "Lidar_slam");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry> ("/lidar/odometry", 1);
    points_sub = nh.subscribe( "/velodyne/data", 1, LidarCallBack );
    
    ros::spin();
}

*/

void ProcessData( const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud )
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::fromROSMsg (*points, *cloud);

    std::cout<< " read sensor msg with seq " << cloud->header.seq << std::endl;
    slam.run( cloud, seq );

    Eigen::Affine3d odom;
    slam.getOdometry( odom );

    publishOdometry( odom );
    seq++;
}



void ReadAndPublishPointCloud()
{

    //bitset<2> foldercount(0);
    //bitset<6> filecount(0);

    int seq = 0;
    int filecount = 0, prev_filecount = 0;
    int numZero = 6;
    int numdigits = 1, div = 0;
    int id = 0;
    while( seq != 22 )
    {
        std::string input_path = "/media/rajath/Rajath/Velodyne/dataset/sequences/" + std::to_string( seq ) + "/velodyne/";

        struct dirent *entry;
        DIR *dir = opendir(input_path.c_str());

        if (dir == NULL) {
            continue;
        }
        filecount = 0;
        while ((entry = readdir(dir)) != NULL) {
            // << endl;
            //std::string infile = input_path + entry->d_name;//input_path + imgnumber + ".bin";
            std::string filename = "";

            
            while( filename.length() < ( numZero - numdigits) )
            {
                filename += "0";
            }
            std::string infile = input_path + filename + std::to_string(filecount) + ".bin";
            std::cout<< infile << std::endl;

            fstream input(infile.c_str(), ios::in | ios::binary);
            if(!input.good()){
                cerr << "Could not read file: " << infile << endl;
                //exit(EXIT_FAILURE);
                numdigits = 1;
                filecount = 0;
                break;
            }
            input.seekg(0, ios::beg);

            pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

            int i;
            for (i=0; input.good() && !input.eof(); i++) {
                pcl::PointXYZI point;
                input.read((char *) &point.x, 3*sizeof(float));
                input.read((char *) &point.intensity, sizeof(float));
                points->push_back(point);
            }
            input.close();

            ProcessData( points );

            //getMapPoints(  )
            displayPointCloud( slam.map );
            /*
            pcl::toROSMsg(*points, output);
            output.header.frame_id = "map";
            output.header.stamp = ros::Time::now();
            output.header.seq = id;
            pub.publish( output );
            
            id++;
            */
            //sensor_msgs::PointCloud2 output;

            //slam.publishMap( output);
            //map_pub.publish( output );

            cout << "Read KTTI point cloud at " << infile << " with " << i << " points " << endl;
            filecount++;
            //prev_filecount = filecount;
            div = filecount;
            numdigits = 0;
            while( div != 0 )
            {
                div /= 10;
                numdigits++;
            }
        }
        seq++;
    }
}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "Lidar_slam");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry> ("/lidar/odometry", 1);
    map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map/points", 1);

    ReadAndPublishPointCloud();
    
    //ros::spin();
}
