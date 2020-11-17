#include "lidarStrat.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lidarStrat");
    ros::NodeHandle n;

    LidarStrat my_lidar_strat(n);

    my_lidar_strat.run();

    return 0;
}
