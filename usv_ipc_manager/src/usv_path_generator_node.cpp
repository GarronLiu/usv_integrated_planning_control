#include "usv_ipc_manager/usv_path_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmpc_tracker");
    ros::NodeHandle nh;
    Usv_Path_Generator tracker(nh);

    ROS_INFO("\033[1;32m----> NMPC Control Loop Started.\033[0m");
   
    ros::spin();

    return 0;
}