#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <mutex>
#include <tf2_eigen/tf2_eigen.h>

class CameraPoseAndCloudPublisher {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pose_pub_, cloud_pub_;
  ros::Subscriber cloud_sub_;
  ros::Timer pose_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string parent_frame_, child_frame_, pose_topic_, cloud_topic_, cloud_out_topic_;
  std::mutex tf_mutex_;

public:
  CameraPoseAndCloudPublisher()
    : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {

    // Load params
    pnh_.param<std::string>("parent_frame", parent_frame_, "world");
    pnh_.param<std::string>("child_frame", child_frame_, "/camera_optical_frame");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/camera/pose");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/camera/depth/points");
    pnh_.param<std::string>("cloud_out_topic", cloud_out_topic_, "/camera/points_world");

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    // cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    // cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CameraPoseAndCloudPublisher::cloudCallback, this);

    pose_timer_ = nh_.createTimer(ros::Duration(0.033), &CameraPoseAndCloudPublisher::poseTimerCallback, this);  // 30 Hz

    ROS_INFO("[Camera Publisher] Initialized.");
  }

  void poseTimerCallback(const ros::TimerEvent&) {
    geometry_msgs::TransformStamped tf_stamped;
    try {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      tf_stamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "[Camera Publisher] TF lookup failed: %s", ex.what());
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = tf_stamped.header.stamp;
    pose.header.frame_id = parent_frame_;
    pose.pose.position.x = tf_stamped.transform.translation.x;
    pose.pose.position.y = tf_stamped.transform.translation.y;
    pose.pose.position.z = tf_stamped.transform.translation.z;
    pose.pose.orientation = tf_stamped.transform.rotation;

    pose_pub_.publish(pose);
  }

  // void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  //   /* Transform the pointcloud from camera_optical_frame to world */
  //   geometry_msgs::TransformStamped tf_stamped;
  //   try {
  //     std::lock_guard<std::mutex> lock(tf_mutex_);
  //     tf_stamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, ros::Time(0));
  //   } catch (tf2::TransformException &ex) {
  //     ROS_WARN_THROTTLE(1.0, "[Camera Publisher] TF lookup failed: %s", ex.what());
  //     return;
  //   }
  //   sensor_msgs::PointCloud2 cloud_out;
  //   try {
  //     // Transform the point cloud
  //     pcl::PointCloud<pcl::PointXYZ> cloud_in;
  //     pcl::fromROSMsg(*msg, cloud_in);
  //     pcl::PointCloud<pcl::PointXYZ> cloud_out_pcl;

  //     Eigen::Affine3d transform = tf2::transformToEigen(tf_stamped.transform);
  //     pcl::transformPointCloud(cloud_in, cloud_out_pcl, transform);

  //     pcl::toROSMsg(cloud_out_pcl, cloud_out);
  //     cloud_out.header = msg->header;
  //     cloud_out.header.frame_id = parent_frame_;

  //     cloud_pub_.publish(cloud_out);
  //   } catch (std::exception& e) {
  //     ROS_ERROR_STREAM("PointCloud transform error: " << e.what());
  //   }
  // }

  void spin() {
    ros::spin();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_pose_publisher");
  CameraPoseAndCloudPublisher node;
  node.spin();
  return 0;
}