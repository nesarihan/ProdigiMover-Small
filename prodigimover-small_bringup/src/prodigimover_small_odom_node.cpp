#include "prodigimover_small_odom.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "prodigimover_small_odom_node");
    ProdigimoverOdom prodigimover;
    ros::spin();
    return 0;
}
