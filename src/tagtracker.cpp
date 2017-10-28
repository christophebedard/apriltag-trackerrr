#include "apriltag_trackerrr/tagtracker.h"

TagTracker::TagTracker(ros::NodeHandle& n)
    : n_(n), it_(n), loop_hz_(LOOP_RATE)
{
    // setup subscribers
    image_sub_ = it_.subscribeCamera("/"+CAMERA_NAMESPACE+"/"+IMAGE_RECT_TOPIC_NAME, 100, &TagTracker::imageCallback, this);
    tag_detect_sub_ = n_.subscribe("/"+CAMERA_NAMESPACE+"/"+TAG_DETECTIONS_TOPIC_NAME, 100, &TagTracker::tagPositionCallback, this);

    // setup publishers
    track_image_pub_ = it_.advertise("/"+CAMERA_NAMESPACE+"/"+"tracking_error", 1);
    joint_state_command_pub_ = n.advertise<sensor_msgs::JointState>(JOINT_STATE_COMMAND_TOPIC_NAME, 100);
    tag_target_pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(TAG_TARGET_POSE_TOPIC_NAME, 10);
}

TagTracker::~TagTracker()
{
}

geometry_msgs::PoseStamped TagTracker::createPoseStampedFromPosYaw(double yaw, std::string frame) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = frame;
    return pose_msg;
}

sensor_msgs::JointState TagTracker::createJointStateFromAngles(std::vector<double> angles) {
    sensor_msgs::JointState state_msg;
    state_msg.position = angles;
    return state_msg;
}

/*===========================
 * Callbacks
 *===========================*/

void TagTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    // convert image to opencv image type
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // update camera model
    cam_model_.fromCameraInfo(cam_info);

    int middle_x = cv_ptr->image.cols / 2;
    int middle_y = cv_ptr->image.rows / 2;
    cv::Point2d middle(middle_x, middle_y);
    //cv::circle(cv_ptr->image, cv::Point(middle_x, middle_y), 50, CV_RGB(255,0,0));

    // draw
    for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
        cv::Point3d pt_cv(tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z);
        cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
        cv::arrowedLine(cv_ptr->image, middle, uv, CV_RGB(255,0,0));
    }


    // output modified video stream
    track_image_pub_.publish(cv_ptr->toImageMsg());
}

void TagTracker::tagPositionCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
    // update vector
    detected_tags_ = msg->detections;
}

/*===========================
 * Update
 *===========================*/

void TagTracker::update() {
    if (!detected_tags_.empty()) {
        // not empty; found tag(s)
        bool found_target_tag = false;
        for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
            //ROS_INFO_STREAM(tag.id);
            if (tag.id == TARGET_TAG_ID) { found_target_tag = true; }
        }

        if (found_target_tag) {
            tf::StampedTransform transform1;
            tf::StampedTransform transform2;
            try {
                tf_listener_.waitForTransform(BASE_TF_NAME, TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), ros::Duration(0.5));
                tf_listener_.lookupTransform(BASE_TF_NAME, TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), transform1);

                tf_listener_.waitForTransform(TILT_TF_NAME, TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), ros::Duration(0.5));
                tf_listener_.lookupTransform(TILT_TF_NAME, TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), transform2);
            } catch (tf::TransformException ex) {
                ROS_ERROR("Error looking up tag transform: %s", ex.what());
            }
            std::vector<double> angles;
            angles.push_back(atan2(transform1.getOrigin().y(), transform1.getOrigin().x()));
            angles.push_back(atan2(transform2.getOrigin().y(), transform2.getOrigin().x()));
            joint_state_command_pub_.publish(createJointStateFromAngles(angles));
            tag_target_pose_pub_.publish(createPoseStampedFromPosYaw(angles[0], BASE_TF_NAME));
            tag_target_pose_pub_.publish(createPoseStampedFromPosYaw(angles[1], TILT_TF_NAME));
        }

    } else {
        // empty; didn't find tag
        //ROS_INFO_STREAM("DID NOT find tag");
    }
}

void TagTracker::spinOnce()
{
    update();
    ros::spinOnce();
}

void TagTracker::spin()
{
    ros::Rate rate(loop_hz_);
    
    while (ros::ok())
    {
        spinOnce();
        
        is_running_slowly_ = !rate.sleep();
        if (is_running_slowly_)
        {
            ROS_WARN("[TAGTRACKER] Loop running slowly.");
        }
    }
}

// ---------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tagtracker");
    ros::NodeHandle n;
    
    //ros::NodeHandle n_p("~");
    //int robot_id;
    //n_p.getParam("robot_id", robot_id);
    
    TagTracker tag_tracker(n);
    
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