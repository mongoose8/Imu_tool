#ifndef IMU_TOOL_H
#define IMU_TOOL_H


#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float64.h>


class ImuTool
 {

  public:

      ImuTool();
      ~ImuTool();
  
    

   void acceleratorCallback(const geometry_msgs::Vector3 & imu_msg_raw);
   void poseCallback(const geometry_msgs::PoseStamped& pose_msg);
   void timeCallback(const std_msgs::Float64& time);
   std_msgs::Float64 getTime();
   


  // void poseCallback(const geometry_msgs::PoseStamped& pose_msg);
  // void calibrate_x(const geometry_msgs::Vector3& imu_msg_raw);
   //void calibrate_y(const geometry_msgs::Vector3& imu_msg_raw);
  // void sampling(const geometry_msgs::Vector3& acc);


protected:
  // boost::shared_ptr<
  sensor_msgs::Imu last_imu_msg_;
  float velocityx;
  float velocityy;
  geometry_msgs::Point pose;
  geometry_msgs::Pose c_pose;
  size_t callback_count_;
  ros::Publisher odometryPublisher_;
  ros::Publisher posePublisher_;
  std::string p_imu;
  std::string p_pose;
  ros::Subscriber ImuSub;
  ros::Subscriber Pos_sub;
  ros::Subscriber time_sub;
  ros::NodeHandle node_;
  tf::Transform robot_pose_transform_;
  bool last_imu;
  bool last_pose_;
  std_msgs::Float64 dt;
  float xvel;
  float yvel;
  float last_xvel;
  float last_yvel;
  float pre_xaccel;
  float pre_yaccel;
  //float curr_posex;
  //float curr_posey;
  geometry_msgs::PoseStamped curr_pose;
  geometry_msgs::PoseStamped last_pose;
  geometry_msgs::Point last_pose_msg_;
  tf::Quaternion robot_pose_quaternion_;
  tf::Point robot_pose_position_;
  std::string p_base_footprint_frame_;
  std::string p_base_stabilized_frame_;
  nav_msgs::Odometry odom_msg_;
  tf::Quaternion orientation_quaternion_;
  tf::TransformBroadcaster* tfB_;
  std::string p_base_frame_;
   std::string p_map_frame_;
   std::string p_odom_frame_;
   std::string rate;
   std::string p_accu;
   std::string p_time;
};

#endif // IMU_TOOL_H
