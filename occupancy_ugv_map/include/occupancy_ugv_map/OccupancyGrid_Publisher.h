#include <ros/ros.h>
#include <boost/multi_array.hpp>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <algorithm>
#include <deque>
#include "occupancy_ugv_map/Consts.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;

typedef sync_policies::ApproximateTime<PointCloud2, Odometry> SyncPolicy;
typedef boost::multi_array<deque<geometry_msgs::Point32>, 2> RawGridArray;

template<class T>
using ApproximateSubscriber = message_filters::Subscriber<T>;

class OccupancyGrid_Publisher
{
private:
    /* data */
    ros::NodeHandle n;
    ros::Publisher gridPublisher;
    const int buff_size = 10;
    int width, height = 0;
    double resolution = 0.1;    //  cell size in meters
    string pointCloud2Input ="";        // topic name
    string odomInput ="";               // topic name
    string occupancyGridOutput ="";     // topic name

    RawGridArray rawGrid;
    ApproximateSubscriber<PointCloud2> *pointCloudSub;
    ApproximateSubscriber<Odometry> *odomSub;
    Synchronizer<SyncPolicy> *sync;
    

public:
    OccupancyGrid_Publisher(ros::NodeHandle n, int width, int height,double resolution, string pointClouIn,string odometryIn,string gridOut);
    void Init();
    void SyncCallback(const PointCloud2::ConstPtr &pc2, const Odometry::ConstPtr &odom);
    ~OccupancyGrid_Publisher();
};
