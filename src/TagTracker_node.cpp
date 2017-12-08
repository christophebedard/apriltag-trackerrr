#include "trackerrr/TagTracker.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "tagtracker");
    ros::NodeHandle n;
    
    ros::NodeHandle n_p("~");
    int dof;
    n_p.getParam("dof", dof);
    
    TagTracker tag_tracker(n, dof);
    
    try
    {
        tag_tracker.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[TAGTRACKER] Runtime error: " << e.what());
        return 1;
    }    
    return 0;
}