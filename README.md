# lidar-slam
Lidar Slam based on PCL-GICP matching

Usage:
        git clone https://github.com/RajathSwaroop/lidar-slam      into the source directory of catkin_ws
        
        catkin_make
        
        source ./devel/setup.bash
        
        roscore
        
        and in another terminal  rosrun lidar-slam lidar-slam

        
TODO:
        Make it more generic with params in config file.
        Improve frame rate of GICP matching.
        Colorize point cloud from Stereo images provided by Kitti Dataset.
        Graph Optimization using g2o.
        3-D bounding box perdication based on color images and Lidar point cloud.
