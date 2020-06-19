#include <ros/ros.h>
#include <ros/console.h>
#include "occupancy_ugv_map/Consts.h"
#include "occupancy_ugv_map/OccupancyGrid_Publisher.h"

using namespace std;


string pcTopicName, ogTopicName, odomTopicName = ""; // input topic and output topic;
int width,height = 0;
double resolution = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_grid_node");
    ros::param::param<string>("~cloud_in", pcTopicName, DEFAULT_IN);
    ros::param::param<string>("~output", ogTopicName, DEFAULT_OUT);
    ros::param::param<string>("~odom",odomTopicName, DEFAULT_ODOM);
    ros::param::param<int>("~width",width, DEFAULT_WIDTH);
    ros::param::param<int>("~height",height, DEFAULT_HEIGHT);
    ros::param::param<double>("~resolution",resolution, DEFAULT_RESOLUTION);
    

    //listener = new tf::TransformListener();

    ros::NodeHandle n;
    OccupancyGrid_Publisher occupancyGrid = OccupancyGrid_Publisher(n,width,height,resolution,pcTopicName,odomTopicName,ogTopicName);
    occupancyGrid.Init();
    ros::spin();
    //delete listener;
    return 0;
}
