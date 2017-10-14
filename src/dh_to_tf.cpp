#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <string>

static const std::string NODE_NAME = "dh_to_tf";
static const int COL_OFFSET = 0;
static const int COL_D      = 1;
static const int COL_A      = 2;
static const int COL_ALPHA  = 3;
static const int COL_TOTAL  = 4;

int joint_id;
std::vector<double> dh_matrix;
std::string frame_prefix, angle_topic_prefix, child_frame, frame, angle_topic;
double offset, d, a, alpha;

void angleCallback(const std_msgs::Float64::ConstPtr& msg){
    double theta = offset + msg->data;

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
    n_p.getParam("joint_id", joint_id);
    n_p.getParam("/frame_prefix", frame_prefix);
    n_p.getParam("/angle_topic_prefix", angle_topic_prefix);
    n_p.getParam("/dh_matrix", dh_matrix);
    
    child_frame = frame_prefix + std::to_string(joint_id - 1);
    frame       = frame_prefix + std::to_string(joint_id);
    angle_topic = angle_topic_prefix + std::to_string(joint_id);

    offset = dh_matrix[COL_TOTAL * (joint_id - 1) + COL_OFFSET];
    d      = dh_matrix[COL_TOTAL * (joint_id - 1) + COL_D];
    a      = dh_matrix[COL_TOTAL * (joint_id - 1) + COL_A];
    alpha  = dh_matrix[COL_TOTAL * (joint_id - 1) + COL_ALPHA];

    ros::Subscriber angle_sub = n.subscribe(angle_topic, 10, &angleCallback);
    
    ros::spin();
    
    return 0;
}