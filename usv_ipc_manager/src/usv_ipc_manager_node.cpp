#include <usv_ipc_manager/usv_ipc_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usv_ipc_manager");
    UsvIpcManager manager;

    ROS_INFO("\033[1;32m----> USV IPC Process Started.\033[0m");
   
     //ros::MultiThreadedSpinner spinner(4);
     //spinner.spin();
     //ros::spin();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
