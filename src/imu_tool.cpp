#include "imu_tool.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>

ImuTool::ImuTool()
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
    private_nh_.param("map_frame", p_map_frame_, std::string("map"));
    private_nh_.param("base_footprint_frame", p_base_footprint_frame_, std::string("base_footprint"));
    private_nh_.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized"));
    private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));
    private_nh_.param("pose_subscribe", p_pose, std::string("pose"));
    private_nh_.param("time_subcriber", p_time, std::string("time"));



    //private_nh_.param("topic", p_imu, std::string("imu_tool"));
    private_nh_.param("Accu", p_accu, std::string("x_acc") );
    ImuSub = node_.subscribe(p_accu, 1000, &ImuTool::acceleratorCallback, this);
    Pos_sub = node_.subscribe(p_pose, 1000, & ImuTool::poseCallback, this);
    time_sub = node_.subscribe(p_time, 1000, & ImuTool::timeCallback, this);
    posePublisher_ = node_.advertise<geometry_msgs::Point>("imu_out_pose", 1, false);
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("odom_p",50);
    last_imu = false;
    last_pose_ = false;

    tfB_ = new tf::TransformBroadcaster();
   // sampling_count = 0;
   // calibration_count = 0;

}



ImuTool::~ImuTool()
{
}



void ImuTool::acceleratorCallback(const geometry_msgs::Vector3& accel)
{



  std_msgs::Float64 delta = ImuTool::getTime();

  if(!last_pose_)
  {
       xvel = accel.x * delta.data;
       yvel = accel.y * delta.data;
       last_xvel = xvel;
       last_yvel = yvel;
       last_pose_msg_.x = last_xvel * delta.data;
       last_pose_msg_.y = last_yvel * delta.data;

     last_pose_ = true;
  }



  xvel = last_xvel + (pre_xaccel+((accel.x-pre_xaccel)/2))*delta.data;
  yvel = last_yvel + (pre_yaccel+((accel.y-pre_yaccel)/2))*delta.data;

  pose.x = last_pose_msg_.x + (pre_xaccel+((xvel-last_xvel)/2))*delta.data;
  pose.y = last_pose_msg_.y + (pre_yaccel+((yvel-last_yvel)/2))*delta.data;

  pre_xaccel = accel.x;
  pre_yaccel = accel.y;
  last_xvel = xvel;
  last_yvel = yvel;
  last_pose_msg_ = pose;








/*    if(!last_imu)
    {
     last_imu_msg_ = imu_msg_raw;
     last_imu = true;
    }

   // velocityx = l_a_v.x + l_l_a.x + ((lin_acc.x - l_l_a.x)/2);
   // velocityy = l_a_v.y + l_l_a.y + ((lin_acc.y - l_l_a.y)/2);
    if(!last_pose_)
    {
        last_pose = curr_pose;
        last_pose_ = true;

    }
     last_pose_msg_.x = last_pose.pose.position.x;
     last_pose_msg_.y = last_pose.pose.position.y;
     x = imu_msg_raw.linear_acceleration.x*cos(theta) + imu_msg_raw.linear_acceleration.y*sin(theta);
     y = imu_msg_raw.linear_acceleration.y*sin(theta) - imu_msg_raw.linear_acceleration.y*cos(theta);

     pose.x = last_pose_msg_.x + x;
     pose.y = last_pose_msg_.y + y;

  //  pose.x = last_pose_msg_.x + last_imu_msg_.angular_velocity.x + ((velocityx - last_imu_msg_.angular_velocity.x)/2);
  //  pose.y = last_pose_msg_.y + last_imu_msg_.angular_velocity.y + ((velocityy - last_imu_msg_.angular_velocity.y)/2);

*/

   posePublisher_.publish(pose);

 /*  if((callback_count_%5)==0)
   {
       odom_msg_.header.stamp = imu_msg_raw.header.stamp;
       odom_msg_.pose.pose.position = last_pose.pose.position;
       odometryPublisher_.publish(odom_msg_);

   }*/
  last_pose_msg_ = pose;
 // last_imu_msg_=imu_msg_raw;

}



void ImuTool::poseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    //boost::mutex::scoped_lock(mutex_);

    curr_pose = pose_msg;

    std::vector<tf::StampedTransform> transforms;
      transforms.resize(2);

      //tf::quaternionMsgToTF(pose_msg->pose.orientation, robot_pose_quaternion_);
      tf::pointMsgToTF(pose_msg.pose.position, robot_pose_position_);

       robot_pose_transform_.setRotation(robot_pose_quaternion_);
        robot_pose_transform_.setOrigin(robot_pose_position_);


        tf::Transform height_transform;
        height_transform.setIdentity();
        height_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

        transforms[0] = tf::StampedTransform(robot_pose_transform_, pose_msg.header.stamp, p_map_frame_, p_base_footprint_frame_);
        transforms[1] = tf::StampedTransform(height_transform, pose_msg.header.stamp, p_base_footprint_frame_, p_base_stabilized_frame_);

        tfB_->sendTransform(transforms);

         // Perform simple estimation of vehicle altitude based on orientation
         if (last_pose_){
           tf::Vector3 plane_normal = tf::Matrix3x3(orientation_quaternion_) * tf::Vector3(0.0, 0.0, 1.0);

           tf::Vector3 last_position;
           tf::pointMsgToTF(last_pose.pose.position, last_position);

           double height_difference =
               (-plane_normal.getX() * (robot_pose_position_.getX() - last_position.getX())
                -plane_normal.getY() * (robot_pose_position_.getY() - last_position.getY())
                +plane_normal.getZ() * last_position.getZ()) / last_position.getZ();

         }

         last_pose = curr_pose;


}

void ImuTool::timeCallback(const std_msgs::Float64 &time)
{
    dt = time;

}

std_msgs::Float64 ImuTool::getTime()
{
    return dt;
}



/*void ImuTool::calibrate_x(const geometry_msgs::Vector3& imu_msg_raw)
{
    if((calibration_count%50)==0)
    {
        xbias = a_x/calibration_count;
        ybias = a_y/calibration_count;
        calibration_count = 0;
        a_x = 0;

    }
     else
    {

        a_x += imu_msg_raw.x;
        a_y += imu_msg_raw.y;
        calibration_count++;
    }



   //return x_Bias;



}*/

/*void ImuTool::calibrate_y(const geometry_msgs::Vector3& imu_msg_raw)
{
    int calibration_count = 0;
   // a_y = 0;

        a_y += imu_msg_raw.y;







  // return y_Bias;



}*/


/*void ImuTool::sampling(const geometry_msgs::Vector3& acc)
{
    int sampling_count = 0;


     acc_x += acc.x;
     acc_y += acc.y;








}*/



