#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

std::string root_frame;
std::string child_frame;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    // Publish transform
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = msg->header;
    odom_tf.header.frame_id = root_frame;
    odom_tf.child_frame_id = child_frame;
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster.sendTransform(odom_tf);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Retrieve node parameters
    nh.param<std::string>("root_frame", root_frame, "odom");
    nh.param<std::string>("child_frame", child_frame, "base_link");

    // Subscribe to odometry message
    ros::Subscriber sub_odom = n.subscribe("input_odom", 10, odomCallback);

    ros::spin();

    return 0;
}
