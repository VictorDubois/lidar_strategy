#include "lidarStrat.h"

int main(int argc, char* argv[])
{
    LidarStrat* my_lidar_strat = new LidarStrat(argc, argv);

    my_lidar_strat->run();

    return 0;
}
