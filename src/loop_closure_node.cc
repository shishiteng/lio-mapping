#include <ros/ros.h>
#include "loop_closure/loop_closure.h"

#include <mutex>
#include <thread>

//for loop closure
std::mutex m_process_;

void CommandProcess(LoopClosure *lp_ptr)
{
    while (1)
    {
        char key = getchar();
        if (key == 'l')
        {
            printf("trigger loop closure...\n");
            m_process_.lock();
            ros::Time stime = ros::Time::now();
            lp_ptr->HandleLoopClosures();
            ROS_INFO("HandleLoopClosures cost: %f", (ros::Time::now() - stime).toSec());
            m_process_.unlock();
        }
        else if (key == 'f')
        {
            printf("force loop closure...\n");
            m_process_.lock();
            ros::Time stime = ros::Time::now();
            lp_ptr->HandleLoopClosures(false);
            ROS_INFO("HandleLoopClosures cost: %f", (ros::Time::now() - stime).toSec());
            m_process_.unlock();
        }
        else if (key == '4')
        {
            printf("4dof loop closure...\n");
            m_process_.lock();
            ros::Time stime = ros::Time::now();
            lp_ptr->Handle4DofLoopClosures();
            ROS_INFO("Handle4DofLoopClosures cost: %f", (ros::Time::now() - stime).toSec());
            m_process_.unlock();
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_closure");
    ros::NodeHandle n("~");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    LoopClosure lp;
    if (!lp.Initialize(n))
    {
        ROS_ERROR("%s: Failed to initialize loop closure processor.", ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    // loop closure trigger
    std::thread keyboard_command_process = std::thread(CommandProcess, &lp);

    ros::spin();

    return EXIT_SUCCESS;
}
