/*
 * This is a Gazebo simulation Plugin of NuBot_Pumbaa tracked vehicle.
 * It is controlled by ROS topic through trackcmd & flipcmd.
 * /NuBot_Pumbaa/nubotcontrol/trackcmd geometry_msgs/Twist linear.x angular.z
 * /NuBot_Pumbaa/nubotcontrol/flipcmd nubot_pumbaa_msg/FlipCmd
 * The core simulation mechanism is based on the brilliant work of Martin Pecka.
 * TrackedVehiclePlugin.hh SimpleTrackedVehiclePlugin.hh 2020/02
 * Contributer: skywalker1941 from NuBot team.
 *
/// \brief An abstract gazebo model plugin for tracked vehicles.
/// \since 8.1
///
/// The plugin processes the following parameters (all have defaults):
/// <steering_efficiency>  Steering efficiency coefficient (0.0 to 1.0).
///                        Default is 0.5.
/// <tracks_separation>  Distance between the centers of the tracks.
///                      Default is 0.4. Implementation may try to autodetect
///                      this value.
/// <max_linear_speed>  Max linear velocity in m/s. Also max track velocity.
///                     Default is 1.0.
/// <max_angular_speed>  Max angular speed in rad/s. Default is 1.0.
/// <track_mu>  Friction coefficient in the first friction direction. If not
///             set, mu of the tracks is not changed, so the values from model
///             definition are used.
/// <track_mu2>  Friction coefficient in the second friction direction. If not
///             set, mu of the tracks is not changed, so the values from model
///             definition are used.
/// <robot_namespace>  Namespace used as a prefix for gazebo topic names.
///                    Default is the name of the model.
///
///
/// \brief A very fast, but also very accurate model of non-deformable tracks
///        without grousers.
/// \since 8.1
///
/// The motion model is based on adjusting motion1 in ODE contact properties
/// and on computing Instantaneous Center of Rotation for a tracked vehicle.
/// A detailed description of the model is given in
/// https://arxiv.org/abs/1703.04316 .
///
/// The plugin processes the following parameters, plus the common parameters
/// defined in TrackedVehiclePlugin.
///
/// <body>  Body of the vehicle to which the two tracks are connected.
/// <left_track>  The left track link's name.
/// <right_track>  The right track link's name.
/// <left_flipper>  The name of a left flipper link.
///     Can appear multiple times.
/// <right_flipper>  The name of a right flipper link.
///     Can appear multiple times.
/// <collide_without_contact_bitmask> Collision bitmask that will be set to
///     the whole vehicle (default is 1u).
*/
#ifndef NUBOT_Pumbaa_GAZEBO_HH
#define NUBOT_Pumbaa_GAZEBO_HH

#include <functional>
#include <vector>
#include <string>
#include <unordered_map>
#include <eigen3/Eigen/Eigen>

#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>
#include <boost/bind.hpp>

#include <gazebo/physics/ode/ode_inc.h>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/ode/contact.h>

#include <gazebo/gazebo.hh>             // the core gazebo header files, including gazebo/math/gzmath.hh
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <sdf/sdf.hh>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>         // Custom Callback Queue
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include "NuBot_Pumbaa_Gazebo_Plugin.hh"

#include "nubot_msgs/base_info.h"
#include "nubot_msgs/base_drive_cmd.h"
#include "math.h"

// This contact.h must be include, it is ralated to the ode
#include "../gazebo_deps_opende/src/joints/contact.h"

namespace gazebo
{
  /// \enum Tracks
  /// \brief Enum for distinguishing between left and right tracks.
  enum class Tracks : bool { LEFT, RIGHT };
}

namespace std   ///Do not understand these code
{
  template <> struct hash<gazebo::Tracks>
  {
    size_t operator() (const gazebo::Tracks &_t) const
    {
      return size_t(_t);
    }
  };
}

namespace gazebo
{
  class NuBotPumbaaGazebo : public ModelPlugin
  {
    public:
      /// \brief Constructor. Will be called firstly
      NuBotPumbaaGazebo();

      /// \brief Destructor
      virtual ~NuBotPumbaaGazebo();

    protected:
      /// \brief Load the controller.
      /// Required by model plugin. Will be called secondly
      /// \param[in] _model Pointer to the model for which the plugin is loaded
      /// \param[in] _sdf Pointer to the SDF for _model
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) ;

      /// \brief Update the controller. It is running every simulation iteration.
      /// [realtime factor] = [realtime update rate] * [max step size].
      virtual void update_child();

      /// \brief Model Reset function.
      /// Not required by model plugin. It is triggered when the world resets.
      virtual void Reset();

    private:

      physics::WorldPtr           world_;             // A pointer to the gazebo world.
      physics::WorldPtr           world_model_;       // Pointer to the world model
      physics::ModelPtr           robot_model_;       // Pointer to the robot model
      sdf::ElementPtr             world_sdf_;         // Pointer to the world sdf file
      sdf::ElementPtr             robot_sdf_;         // Pointer to the robot sdf file

      ros::NodeHandle*            rosnode_;           // A pointer to the ROS node.
      ros::Subscriber             DriveCmd_sub_;      // listen to the Tracks & Flippers command
      ros::Subscriber             DriveAutoCmd_sub_;      // listen to the Tracks & Flippers command
      ros::Publisher              debug_pub_;
      ros::Publisher              RobotState_pub_;   // Publish the pose of the robot
      ros::Publisher              RobotPose_pub_;   // Publish the pose of the robot
      ros::Publisher              LidarPose_pub_;   // Publish the pose of the lidar

      boost::thread               message_callback_queue_thread_;     // Thead object for the Ros running callback Thread.
      boost::thread               service_callback_queue_thread_;
      boost::mutex                msgCB_lock_;        // A mutex to lock access to fields that are used in ROS message callbacks
      boost::mutex                srvCB_lock_;        // A mutex to lock access to fields that are used in ROS service callbacks
      ros::CallbackQueue          message_queue_;     // Custom Callback Queue. Details see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
      ros::CallbackQueue          service_queue_;     // Custom Callback Queue
      event::ConnectionPtr        update_connection_;         // Pointer to the update event connection

      gazebo_msgs::ModelStates    model_states_;      // Container for the ModelStates msg
      //model_state                 robot_state_;

      std::string                 robot_name_;        // robot name.
      std::string                 world_name_;        // world name.

      double                      noise_scale_;       // scale of gaussian noise, not used yet
      double                      noise_rate_;        // how frequent the noise generates, not used yet
      double                      tracksSeparation;   // Parameter of tracks
      double                      steeringEfficiency; // Parameter of tracks
      double                      maxLinearSpeed;     // Parameter of tracks
      double                      maxAngularSpeed;    // Parameter of tracks
      double                      flip_p;             // Parameter of flippers position control
      double                      flip_i;             // Parameter of flippers position control
      double                      flip_d;             // Parameter of flippers position control
      double                      flip_initial;       // Parameter of flippers position control
      double                      flip_vel_p;             // Parameter of flippers position control
      double                      flip_vel_i;             // Parameter of flippers position control
      double                      flip_vel_d;             // Parameter of flippers position control
      const double flip_relax = 3.1416/180;
      double flip_err[4] = {0,0,0,0};
      double flip_err_last[4] = {0,0,0,0};
      double flip_integral[4] = {0,0,0,0};
      double flip_speed[4] = {0,0,0,0};
      const double flip_speed_limit = 3.1416*60/180;
      const float main_velocity_trans = 0.1*2*3.1416/25/60;  // 2pi/减速机减速比/60
      const float fin_rate_trans = 2*3.1416/(64*40/30)/60;  // 2pi/减速机减速比/60
      const float angle2radian = 3.1416/180; // pi/180

      Eigen::Quaterniond base_quaternion, lidar_quaternion;
      Eigen::Vector3d base_position, lidar_position;
      Eigen::Matrix4d base_matrix = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d lidar_matrix = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d base_lidar_matrix = Eigen::Matrix4d::Identity();

      /// \brief Friction coefficient in the first friction direction.
      sdf::ParamPtr               trackMu;
      /// \brief Friction coefficient in the second friction direction.
      sdf::ParamPtr               trackMu2;

      /// \brief Most of the code below are learned from Martin Peica's work in 2020/02.
      /// TrackedVehiclePlugin.hh SimpleTrackedVehiclePlugin.hh

      /// \brief Body of the robot.
      physics::LinkPtr body_;

      /// \brief Textual lowercase names of the tracks.
      std::unordered_map<Tracks, std::string> trackName_;

      /// \brief The tracks controlled by this plugin.
      std::unordered_map<Tracks, physics::LinkPtr> trackLink_;

      /// \brief Desired velocities of the tracks.
      std::unordered_map<Tracks, double> trackVelocity_;

      /// \brief Pointer of the flippers joints.
      physics::JointPtr front_left_j;
      physics::JointPtr front_right_j;
      physics::JointPtr rear_left_j;
      physics::JointPtr rear_right_j;

      /// \brief This bitmask will be set to the whole vehicle body.
      unsigned int collideWithoutContactBitmask;

      /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
      virtual double GetSteeringEfficiency();

      /// \brief Distance between the centers of the tracks.
      virtual double GetTracksSeparation();

      /// \brief contactManager
      physics::ContactManager *contactManager;

      /// \brief Set steering efficiency coefficient (between 0.0 and 1.0).
      /// \param[in] _steeringEfficiency The new steering efficiency.
      //virtual void SetSteeringEfficiency(double _steeringEfficiency);

      /// \brief Friction coefficient in the first friction direction.
      virtual boost::optional<double> GetTrackMu();

      /// \brief Friction coefficient in the first friction direction.
      /// \param[in] _mu The new coefficient.
      //virtual void SetTrackMu(double _mu);

      /// \brief Friction coefficient in the second friction direction.
      virtual boost::optional<double> GetTrackMu2();

      /// \brief Friction coefficient in the second friction direction.
      /// \param[in] _mu2 The new coefficient.
      //virtual void SetTrackMu2(double _mu2);

      /// \brief Update surface parameters of the tracks to correspond to the
      ///        values set in this plugin.
      virtual void UpdateTrackSurface();

      /// \brief Set mu and mu2 of all collisions of the given link to values
      ///        given by GetTrackMu() and GetTrackMu2().
      /// \param[in] _link The link whose "mu"s are to be set.
      void SetLinkMu(const physics::LinkPtr &_link);

      /// \brief Set new target velocity for the tracks based on the desired
      /// body motion.
      /// \param[in] _linear Desired linear velocity of the vehicle.
      /// \param[in] _angular Desired angular velocity of the vehicle.
      void SetBodyVelocity(double _linear, double _angular);  // TODO peci1: make virtual

      /// \brief Set new target velocity for the tracks.
      /// Descendant classes need to implement this function.
      /// Do not call this function directly, instead call SetTrackVelocity().
      /// \param[in] _left Velocity of left track.
      /// \param[in] _right Velocity of right track.
      virtual void SetTrackVelocityImpl(double _left, double _right);

      /// \brief Set new target velocity for the tracks.
      /// Descendant classes need to implement this function.
      /// \param[in] _left Velocity of left track.
      /// \param[in] _right Velocity of right track.
      virtual void SetTrackVelocity(double _left, double _right);

      /// \brief Set new target position for the flippers.
      /// \param[in] _frontleft Desired angular position of the front_left flipper.
      /// \param[in] _frontright Desired angular position of the front_right flipper.
      /// \param[in] _rearleft Desired angular position of the rear_left flipper.
      /// \param[in] _rearright Desired angular position of the rear_right flipper.
      void SetFlipAngle(double _frontleft, double _frontright, double _rearleft,
                       double _rearright);  // TODO: make virtual
      void SetFlipAngle_PID(double _frontleft, double _frontright, double _rearleft,
                       double _rearright);  // TODO: make virtual

      /// \brief Set new target position for the flippers.
      /// \param[in] _frontleft Desired angular position of the front_left flipper.
      /// \param[in] _frontright Desired angular position of the front_right flipper.
      /// \param[in] _rearleft Desired angular position of the rear_left flipper.
      /// \param[in] _rearright Desired angular position of the rear_right flipper.
      void SetFlipVelocity(double _frontleft, double _frontright, double _rearleft,
                       double _rearright);  // TODO: make virtual

      /// \brief Set collide categories and bits of all geometries to the
      ///        required values.
      /// This is a workaround for https://bitbucket.org/osrf/gazebo/issues/1855 .
      void SetGeomCategories();

      /// \brief ROS Functions
      ///  
      /// \brief Publish messages to nubot_control node
      void message_publish(void);

      /// \brief PumbaaCmd message CallBack function
      /// \param[in] _msg shared pointer that is used to set the Tracks velocity & Flippers angle
      void Drive_Cmd_CB(const nubot_msgs::base_drive_cmd::ConstPtr &_msg);
      void Drive_Auto_Cmd_CB(const geometry_msgs::Twist::ConstPtr &_msg);

      /// \brief ROS Custom message callback queue thread
      void message_queue_thread();

      /// \brief Compute and apply the forces that make the tracks move.
      void DriveTracks(/*const common::UpdateInfo &_unused*/);

    /// \brief Compute the direction of friction force in given contact point.
    /// \param[in] _linearSpeed Linear speed of the vehicle.
    /// \param[in] _angularSpeed Angular speed of the vehicle.
    /// \param[in] _drivingStraight Whether the vehicle should steer.
    /// \param[in] _bodyPose Pose of the vehicle body.
    /// \param[in] _bodyYAxisGlobal Direction of the y-axis of the body in
    ///            world frame.
    /// \param[in] _centerOfRotation Center of the circle the vehicle
    ///            follows (Inf/-Inf if driving straight).
    /// \param[in] _contactWorldPosition World position of the contact point.
    /// \param[in] _contactNormal Corrected contact normal (pointing inside
    ///            the track).
    /// \param[in] _beltDirection World-frame forward direction of the belt.
    /// \return Direction of the friction force in world frame.
    protected: ignition::math::Vector3d ComputeFrictionDirection(
      double _linearSpeed, double _angularSpeed,
      bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
      const ignition::math::Vector3d &_bodyYAxisGlobal,
      const ignition::math::Vector3d &_centerOfRotation,
      const ignition::math::Vector3d &_contactWorldPosition,
      const ignition::math::Vector3d &_contactNormal,
      const ignition::math::Vector3d &_beltDirection) const;

    /// \brief Compute the velocity of the surface motion in all contact points.
    /// \param[in] _beltSpeed The desired belt speed.
    /// \param[in] _beltDirection Forward direction of the belt.
    /// \param[in] _frictionDirection First friction direction.
    protected: double ComputeSurfaceMotion(double _beltSpeed,
      const ignition::math::Vector3d &_beltDirection,
      const ignition::math::Vector3d &_frictionDirection) const;

    /// \brief Category for the non-track parts of the robot.
    protected: static const unsigned int ROBOT_CATEGORY = 0x10000000;
    /// \brief Category for tracks.
    protected: static const unsigned int BELT_CATEGORY = 0x20000000;
    /// \brief Category for all items on the left side.
    protected: static const unsigned int LEFT_CATEGORY = 0x40000000;

    /// \class ContactIterator
    /// \brief An iterator over all contacts between two geometries.
    class ContactIterator : std::iterator<std::input_iterator_tag, dContact>
    {
      /// \brief The contact to return as the next element.
      private: pointer currentContact;
      /// \brief Index of the last examined joint.
      private: size_t jointIndex;
      /// \brief The body the contact should belong to.
      private: dBodyID body;
      /// \brief The geometries to search contacts for.
      private: dGeomID geom1, geom2;
      /// \brief True if at least one value has been returned.
      private: bool initialized;

      // Constructors.
      public: ContactIterator();
      public: explicit ContactIterator(bool _initialized);
      public: ContactIterator(const ContactIterator &_rhs);
      public: ContactIterator(dBodyID _body, dGeomID _geom1, dGeomID _geom2);

      /// \brief Use to "instantiate" the iterator from user code
      public: static ContactIterator begin(dBodyID _body, dGeomID _geom1,
                                       dGeomID _geom2);
      public: static ContactIterator end();

      /// \brief Finding the next element; this is the main logic.
      public: ContactIterator operator++();

      // Operators. It is required to implement them in iterators.
      public: bool operator==(const ContactIterator &_rhs);
      public: ContactIterator &operator=(const ContactIterator &_rhs);
      public: ContactIterator operator++(int _unused);
      public: reference operator*();
      public: pointer operator->();
      public: pointer getPointer();
      public: bool operator!=(const ContactIterator &_rhs);
    };
  };
}
#endif
