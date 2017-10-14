#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

static const std::string NODE_NAME = "dh_to_tf";

std::string child_frame, frame, angle_topic;
double d, a, alpha;

void angleCallback(const std_msgs::Float64::ConstPtr& msg){
    double theta = msg->data;

    tf::Matrix3x3 rot(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
                      sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                               0,             sin(alpha),             cos(alpha));
    tf::Vector3 trans(a*cos(theta),
                      a*sin(theta),
                                 d);
    tf::Transform transform(rot, trans);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child_frame, frame));
}

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    n_p.getParam("child_frame", child_frame);
    n_p.getParam("frame", frame);
    n_p.getParam("angle_topic", angle_topic);
    n_p.getParam("d", d);
    n_p.getParam("a", a);
    n_p.getParam("alpha", alpha);

    ros::Subscriber angle_sub = n.subscribe(angle_topic, 10, &angleCallback);
    
    ros::spin();
    
    return 0;
}