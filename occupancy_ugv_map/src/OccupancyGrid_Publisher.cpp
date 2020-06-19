#include "occupancy_ugv_map/OccupancyGrid_Publisher.h"

OccupancyGrid_Publisher::OccupancyGrid_Publisher(ros::NodeHandle nodeHandle, int _width, int _height, double _resolution, string pointClouIn, string odometryIn, string gridOut) : rawGrid(boost::extents[_width][_height])
{
    n = nodeHandle;
    width = _width;
    height = _height;
    resolution = _resolution;
    pointCloud2Input = pointClouIn;
    odomInput = odometryIn;
    occupancyGridOutput = gridOut;
    gridPublisher = n.advertise<nav_msgs::OccupancyGrid>(gridOut, 1000);
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            rawGrid[i][j] = deque<geometry_msgs::Point32>();
        }
    }
}
void OccupancyGrid_Publisher::Init()
{
    pointCloudSub = new ApproximateSubscriber<PointCloud2>(n, pointCloud2Input, buff_size);
    odomSub = new ApproximateSubscriber<Odometry>(n, odomInput, buff_size);
    sync = new Synchronizer<SyncPolicy>(SyncPolicy(buff_size), *pointCloudSub, *odomSub);
    sync->registerCallback(boost::bind(&OccupancyGrid_Publisher::SyncCallback, this, _1, _2));
}

void OccupancyGrid_Publisher::SyncCallback(sensor_msgs::PointCloud2ConstPtr const &msg, const Odometry::ConstPtr &odom)
{
    OccupancyGrid grid;
    geometry_msgs::Pose mapOrigin = geometry_msgs::Pose();
    geometry_msgs::Point robotPos = odom.get()->pose.pose.position;

    mapOrigin.position = robotPos;
    mapOrigin.position.x -= resolution * (width / 2);
    mapOrigin.position.y -= resolution * (height / 2);

    grid.header.frame_id = "world";
    grid.header.stamp = ros::Time::now();

    grid.info.map_load_time = ros::Time::now();
    grid.info.height = height;
    grid.info.width = width;
    grid.info.resolution = resolution;
    grid.info.origin = mapOrigin;

    vector<int8_t> mapData = vector<int8_t>(width * height, 0); // Occupancy Grid data.

    sensor_msgs::PointCloud2ConstIterator<float>    iter_x(*msg, "x"),
                                                    iter_y(*msg, "y"),
                                                    iter_z(*msg, "z");
    auto origin = geometry_msgs::Point32();
    origin.x = floor(width / 2);
    origin.y = floor(height / 2); // map world's (0,0,0) to map center

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            rawGrid[i][j].clear();
        }
    }

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)
    {
        // Process here data
        geometry_msgs::Point32 point;
        point.x = it[0];
        point.y = it[1];
        point.z = it[2];
        int x = (int)(floor(point.x / resolution) + origin.x) % width; // translate to map center
        int y = (int)(floor(point.y / resolution) + origin.y) % height;
        if (!((x >= 0 && x < width) && (y >= 0 && y < height))) // check whether index is in right bound 0 <= (x,y) < (width,height)
        {
            // if new indeces out of array just ignore
            continue;
        }
        auto GridCell = &rawGrid[x][y];
        if (GridCell->size() >= MAX_CELL_CAPACITY)
        {
            GridCell->pop_front();
        }
        GridCell->push_back(point);
    }

    // Ascending sort by Z component of the point
    for (size_t y = 0; y < height; y++)
    {
        for (size_t x = 0; x < width; x++)
        {
            auto GridCell = &rawGrid[x][y];
            if (GridCell->size() > 0)
            {
                sort(GridCell->begin(), GridCell->end(), [](const geometry_msgs::Point32 &pt1, const geometry_msgs::Point32 &pt2) {
                    return pt1.z < pt2.z;
                });
            }
        }
    } // End loop
    
    for (size_t y = 0; y < height; y++)
    {
        for (size_t x = 0; x < width; x++)
        {
            auto GridCell = rawGrid[x][y];
            if (GridCell.size() > 0) // replace 0 to some value
            {
                int Cx = x;
                int Cy = y;

                double d = sqrt(pow(Cx * resolution + resolution / 2, 2) + pow(Cy * resolution + resolution / 2, 2));
                double a = max<double>(DESIRE_ATTITUDE, d * (MAX_SLOPE));
                double Zmax = max<double>(VEHICLE_GROUND_CLEARANCE, d * sin(a));

                geometry_msgs::Point32 lowerPointInCell = GridCell.front();
                geometry_msgs::Point32 higherPointInCell = GridCell.back();

                if (higherPointInCell.z - lowerPointInCell.z < MIN_HEIGHT_DIFFERENCE)
                {
                    mapData[x + y * height] = CELL_FREE;
                    continue;
                }
                if (lowerPointInCell.z > Zmax || higherPointInCell.z < -Zmax)
                {
                    mapData[x + y * height] = CELL_OCCUPIED;
                    continue;
                }

                if (higherPointInCell.z - lowerPointInCell.z > VEHICLE_HEIGHT)
                {
                    // NOT EVALUATING;
                    mapData[x + y * height] = CELL_UNKNOWN;
                    continue;
                }

                mapData[x + y * height] = CELL_FREE; 
                continue;
                // Add test for ground and overhanging objects
            }
        }
    }

    grid.data = mapData;
    gridPublisher.publish(grid);
}
OccupancyGrid_Publisher::~OccupancyGrid_Publisher()
{
    delete sync;
    delete pointCloudSub;
    delete odomSub;
    // delete rawGrid;
}