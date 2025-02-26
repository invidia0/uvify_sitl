#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "camera_pose_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string parent_frame, child_frame, pose_topic;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Print the namespace
    ROS_INFO("[Camera Pose Publisher] > Namespace: %s", nh.getNamespace().c_str());

    pnh.getParam("parent_frame", parent_frame);
    pnh.getParam("child_frame", child_frame);
    pnh.getParam("pose_topic", pose_topic);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
    ros::Rate loop_rate(30);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = parent_frame;

    ROS_INFO("[Camera Pose Publisher] > Camera pose publisher started!");
    ROS_INFO("[Camera Pose Publisher] > Parent frame: %s", parent_frame.c_str());
    ROS_INFO("[Camera Pose Publisher] > Child frame: %s", child_frame.c_str());
    ROS_INFO("[Camera Pose Publisher] > Pose topic: %s", pose_topic.c_str());

    while (ros::ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));

          pose.header.stamp = transformStamped.header.stamp;
          pose.pose.position.x = transformStamped.transform.translation.x;
          pose.pose.position.y = transformStamped.transform.translation.y;
          pose.pose.position.z = transformStamped.transform.translation.z;
          pose.pose.orientation = transformStamped.transform.rotation;

          pub.publish(pose);
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        loop_rate.sleep();
    }

    return 0;
};