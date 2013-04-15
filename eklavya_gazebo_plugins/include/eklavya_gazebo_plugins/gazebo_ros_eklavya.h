#ifndef GAZEBO_ROS_EKLAVYA_H
#define GAZEBO_ROS_EKLAVYA_H

#include "physics/physics.h"
#include "physics/PhysicsTypes.hh"
#include "sensors/SensorTypes.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace gazebo
{
  class GazeboRosEklavya : public ModelPlugin
  {
    public: 
      GazeboRosEklavya();
      virtual ~GazeboRosEklavya();
          
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();
  
    private:
      void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);


      /// Parameters
      std::string node_namespace_;
      std::string left_wheel_joint_name_;
      std::string right_wheel_joint_name_;
      std::string front_castor_wheel_joint_name_;
      std::string front_castor_swivel_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;


      ros::NodeHandle *rosnode_;
      //ros::Service operating_mode_srv_;
      //ros::Service digital_output_srv_;
  
      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;
  
      ros::Subscriber cmd_vel_sub_;

      physics::WorldPtr my_world_;
      physics::ModelPtr my_parent_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];

      bool set_joints_[4];
      physics::JointPtr joints_[4];
	  event::ConnectionPtr updateConnection;

      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      void spin();
      boost::thread *spinner_thread_;

  };
}
#endif
