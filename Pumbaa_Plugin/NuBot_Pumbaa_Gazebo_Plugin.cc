/*
 * This is a Gazebo simulation Plugin of NuBot_Pumbaa tracked vehicle.
 * It is controlled by ROS topic through trackcmd & flipcmd.
 * /NuBot_Pumbaa/nubotcontrol/trackcmd geometry_msgs/Twist linear.x angular.z
 * /NuBot_Pumbaa/nubotcontrol/flipcmd nubot_pumbaa_msg/FlipCmd
 * The core simulation mechanism is based on the brilliant work of Martin Pecka.
 * TrackedVehiclePlugin.cc SimpleTrackedVehiclePlugin.cc 2020/02
 * Contributer: skywalker1941 from NuBot team.
*/
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "NuBot_Pumbaa_Gazebo_Plugin.hh"

namespace std       ///Do not understand these code
{
  template<class T>
  class hash<boost::shared_ptr<T>>
  {
    public: size_t operator()(const boost::shared_ptr<T>& key) const
    {
      return (size_t)key.get();
    }
  };
}

namespace gazebo    ///Do not understand these code
{
using namespace std;
using namespace physics;
/// \brief This is a temporary workaround to keep ABI compatibility in
/// Gazebo 9. It should be deleted starting with Gazebo 10.
/// Set a map from the "body_" to tracks, including name link velocity
unordered_map<LinkPtr, unordered_map<Tracks, Link_V> > globalTracks;
}

using namespace gazebo;
using namespace std;
GZ_REGISTER_MODEL_PLUGIN(NuBotPumbaaGazebo)

NuBotPumbaaGazebo::NuBotPumbaaGazebo()
{
  // Variables initialization
  trackName_[Tracks::LEFT] = "left";
  trackName_[Tracks::RIGHT] = "right";
}

NuBotPumbaaGazebo::~NuBotPumbaaGazebo()
{
  update_connection_->~Connection();
  // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish.
  message_queue_.clear();
  //service_queue_.clear();
  // Disable the queue, meaning any calls to addCallback() will have no effect.
  message_queue_.disable();
  //service_queue_.disable();
  rosnode_->shutdown();                     // This MUST BE CALLED before thread join()!!
  message_callback_queue_thread_.join();
  //service_callback_queue_thread_.join();

  delete rosnode_;
  //delete obs_;

  if (body_ != nullptr)
  {
    if (globalTracks.find(body_) != globalTracks.end())
    {
      globalTracks.erase(body_);
    }
  }
}

void NuBotPumbaaGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // This Plugin is placed in the NuBot_Pumbaa.sdf, _model represent the robot. Get the model.
  GZ_ASSERT(_model, "NuBotPumbaaPlugin _model pointer is NULL");
  //robot_model_ means the pointer to the NuBot_Pumbaa robot, related to .sdf file
  robot_model_ = _model;
  //world_model_ means the environments where NuBot_Pumbaa robot simulating, related to .world file.
  world_model_ = robot_model_->GetWorld();

  GZ_ASSERT(robot_model_, "NuBotPumbaaPlugin robot_model_ pointer is NULL");
  GZ_ASSERT(world_model_, "NuBotPumbaaPlugin world_model_ pointer is NULL");

  world_name_ = world_model_->Name();
  robot_name_ = robot_model_->GetName();

  gzmsg << "NuBotPumbaaGazebo_Plugin: world_model_ name is "
        << world_name_ << std::endl;
  gzmsg << "NuBotPumbaaGazebo_Plugin: robot_model_ name is "
        << robot_name_ << std::endl;

  if (world_model_->Physics()->GetType() != "ode")
  {
    gzerr << "Tracked vehicle simulation works only with ODE." << std::endl;
   throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed. ODE Problem");
  }

  // Get the robot's .sdf file name.
  robot_sdf_ = _sdf;
  GZ_ASSERT(_sdf, "NuBotPumbaaGazebo_Plugin robot_sdf_ pointer is NULL");

  /// Load the "track_mu" "track_mu2"
  ///          "tracksSeparation" "steeringEfficiency"
  ///          "maxLinearSpeed" "maxAngularSpeed"
  ///          "flip_p" "flip_i" "flip_d" "flip_initial"
  /// form .sdf file
  if (robot_sdf_->HasElement("track_mu"))
  {
    trackMu = robot_sdf_->GetElement("track_mu")->GetValue();
  }
  if (robot_sdf_->HasElement("track_mu2"))
  {
    trackMu2 = robot_sdf_->GetElement("track_mu2")->GetValue();
  }
  if (robot_sdf_->HasElement("tracksSeparation"))
  {
    tracksSeparation = std::stod(robot_sdf_->GetElement("tracksSeparation")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("steeringEfficiency"))
  {
    steeringEfficiency = std::stod(robot_sdf_->GetElement("steeringEfficiency")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("maxLinearSpeed"))
  {
    maxLinearSpeed = std::stod(robot_sdf_->GetElement("maxLinearSpeed")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("maxAngularSpeed"))
  {
    maxAngularSpeed = std::stod(robot_sdf_->GetElement("maxAngularSpeed")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("flip_p"))
  {
    flip_p = std::stod(robot_sdf_->GetElement("flip_p")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("flip_i"))
  {
    flip_i = std::stod(robot_sdf_->GetElement("flip_i")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("flip_d"))
  {
    flip_d = std::stod(robot_sdf_->GetElement("flip_d")
                         ->GetValue()->GetDefaultAsString());
  }
  if (robot_sdf_->HasElement("flip_initial"))// the initial position of flippers
  {
    flip_initial = std::stod(robot_sdf_->GetElement("flip_initial")
                         ->GetValue()->GetDefaultAsString());
  }

  // Check the basic parameters
  if (steeringEfficiency <= 0.)
      throw std::runtime_error("Steering efficiency must be positive");
  if (tracksSeparation <= 0.)
      throw std::runtime_error("Tracks separation must be positive");
  if (maxLinearSpeed <= 0.)
      throw std::runtime_error("Maximum linear speed must be positive");
  if (maxAngularSpeed < 0.)
      throw std::runtime_error("Maximum angular speed must be non-negative");

  // Check the "body" "left_track" "right_track" element
  if (!robot_sdf_->HasElement("body"))
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <body> tag missing." << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }

  if (!robot_sdf_->HasElement("left_track"))
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <left_track> tag missing."
          << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }

  if (!robot_sdf_->HasElement("right_track"))
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <right_track> tag missing."
          << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }

  // Load the "body" link
  body_ = robot_model_->GetLink(
          robot_sdf_->GetElement("body")->Get<std::string>());
  if (body_ == nullptr)
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <body> link does not exist."
          << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }
  else
  {
    gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added robot body link "
          << body_->GetName() << std::endl;
  }
  // Set the body_ link pair into globalTracks
  globalTracks.emplace(body_,
          std::unordered_map<Tracks, physics::Link_V>());
  auto& gtracks = globalTracks.at(body_);

  // Load the "left_track" Link
  trackLink_[Tracks::LEFT] = robot_model_->GetLink(
          robot_sdf_->GetElement("left_track")->Get<std::string>());
  gtracks[Tracks::LEFT].push_back(trackLink_[Tracks::LEFT]);
  if (gtracks[Tracks::LEFT].at(0) == nullptr)
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <left_track> link does not exist."
          << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }
  else
  {
    gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added left track link "
          << gtracks[Tracks::LEFT].at(0)->GetName() << std::endl;
  }

  // Load the "right_track" Link
  trackLink_[Tracks::RIGHT] = robot_model_->GetLink(
            robot_sdf_->GetElement("right_track")->Get<std::string>());
  gtracks[Tracks::RIGHT].push_back(trackLink_[Tracks::RIGHT]);
  if (gtracks[Tracks::RIGHT].at(0) == nullptr)
  {
    gzerr << "NuBotPumbaaGazebo_Plugin: <right_track> link does not exist."
          << std::endl;
    throw std::runtime_error("NuBotPumbaaGazebo_Plugin: Load() failed.");
  }
  else
  {
    gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added right track link "
          << gtracks[Tracks::RIGHT].at(0)->GetName() << std::endl;
  }

  // Load the "left_flipper_track" Link, relate them to the "left_track"
  if (robot_sdf_->HasElement("left_flipper"))
  {
    auto flipper = robot_sdf_->GetElement("left_flipper");
    while (flipper)
    {
      const auto flipperName = flipper->Get<std::string>();
      const auto flipperLink = robot_model_->GetLink(flipperName);
      if (flipperLink == nullptr)
      {
        gzerr << "NuBotPumbaaGazebo_Plugin: <left_flipper> _track link '"
              << flipperName << "' does not exist." << std::endl;
      }
      else
      {
        gtracks[Tracks::LEFT].push_back(flipperLink);
        gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added left flipper track"
                 "link '" << flipperName << "'" << std::endl;
      }
      flipper = flipper->GetNextElement("left_flipper");
    }
  }

  // Load the "right_flipper_track" Link, relate them to the "right_track"
  if (robot_sdf_->HasElement("right_flipper"))
  {
    auto flipper = robot_sdf_->GetElement("right_flipper");
    while (flipper)
    {
      const auto flipperName = flipper->Get<std::string>();
      const auto flipperLink = robot_model_->GetLink(flipperName);
      if (flipperLink == nullptr)
      {
        gzerr << "NuBotPumbaaGazebo_Plugin: <right_flipper> _track link '"
              << flipperName << "' does not exist." << std::endl;
      }
      else
      {
        gtracks[Tracks::RIGHT].push_back(flipperLink);
        gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added right flipper track"
                 "link '" << flipperName << "'" << std::endl;
      }
      flipper = flipper->GetNextElement("right_flipper");
    }
  }

  // Load the "front_left_flipper" rotate joint
  if (robot_sdf_->HasElement("front_left_flipper_j"))
  {
    const auto flipperName_j = robot_sdf_->GetElement("front_left_flipper_j")
            ->Get<std::string>();
    const auto flipperJoint_name = robot_model_->GetScopedName()+"::"+flipperName_j;
    const auto flipperJoint = robot_model_->GetJoint(flipperJoint_name);
    if (flipperJoint == nullptr)
    {
      gzerr << "NuBotPumbaaGazebo_Plugin: <front_left_flipper_j> joint '"
            << flipperName_j << "' does not exist." << std::endl;
    }
    else
    {
      // Set up the initial PID parameter & position
      front_left_j = flipperJoint;
      robot_model_->GetJointController()->SetPositionPID(
                  front_left_j->GetScopedName(),common::PID(flip_p, flip_i, flip_d));
      robot_model_->GetJointController()->SetPositionTarget(
                  front_left_j->GetScopedName(), -flip_initial);
      gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added joint '"
            << flipperName_j << "'" << std::endl;
    }
  }

  // Load the "rear_left_flipper" rotate joint
  if (robot_sdf_->HasElement("rear_left_flipper_j"))
  {
    const auto flipperName_j = robot_sdf_->GetElement("rear_left_flipper_j")
            ->Get<std::string>();
    const auto flipperJoint_name = robot_model_->GetScopedName()+"::"+flipperName_j;
    const auto flipperJoint = robot_model_->GetJoint(flipperJoint_name);
    if (flipperJoint == nullptr)
    {
      gzerr << "NuBotPumbaaGazebo_Plugin: <rear_left_flipper_j> joint '"
            << flipperName_j << "' does not exist." << std::endl;
    }
    else
    {
      // Set up the initial PID parameter & position
      rear_left_j = flipperJoint;
      robot_model_->GetJointController()->SetPositionPID(
                  rear_left_j->GetScopedName(),common::PID(flip_p, flip_i, flip_d));
      robot_model_->GetJointController()->SetPositionTarget(
                  rear_left_j->GetScopedName(), flip_initial);
      gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added joint '"
            << flipperName_j << "'" << std::endl;
    }
  }

  // Load the "front_right_flipper" rotate joint
  if (robot_sdf_->HasElement("front_right_flipper_j"))
  {
    const auto flipperName_j = robot_sdf_->GetElement("front_right_flipper_j")
            ->Get<std::string>();
    const auto flipperJoint_name = robot_model_->GetScopedName()+"::"+flipperName_j;
    const auto flipperJoint = robot_model_->GetJoint(flipperJoint_name);
    if (flipperJoint == nullptr)
    {
      gzerr << "NuBotPumbaaGazebo_Plugin: <front_right_flipper_j> joint '"
            << flipperName_j << "' does not exist." << std::endl;
    }
    else
    {
      // Set up the initial PID parameter & position
      front_right_j = flipperJoint;
      robot_model_->GetJointController()->SetPositionPID(
                  front_right_j->GetScopedName(),common::PID(flip_p, flip_i, flip_d));
      robot_model_->GetJointController()->SetPositionTarget(
                  front_right_j->GetScopedName(), -flip_initial);
      gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added joint '"
            << flipperName_j << "'" << std::endl;
    }
  }

  // Load the "rear_right_flipper" rotate joint
  if (robot_sdf_->HasElement("rear_right_flipper_j"))
  {
    const auto flipperName_j = robot_sdf_->GetElement("rear_right_flipper_j")
            ->Get<std::string>();
    const auto flipperJoint_name = robot_model_->GetScopedName()+"::"+flipperName_j;
    const auto flipperJoint = robot_model_->GetJoint(flipperJoint_name);
    if (flipperJoint == nullptr)
    {
      gzerr << "NuBotPumbaaGazebo_Plugin: <rear_right_flipper_j> joint '"
            << flipperName_j << "' does not exist." << std::endl;
    }
    else
    {
      // Set up the initial PID parameter & position
      rear_right_j = flipperJoint;
      robot_model_->GetJointController()->SetPositionPID(
                  rear_right_j->GetScopedName(),common::PID(flip_p, flip_i, flip_d));
      robot_model_->GetJointController()->SetPositionTarget(
                  rear_right_j->GetScopedName(), flip_initial);
      gzmsg << "NuBotPumbaaGazebo_Plugin: Successfully added joint '"
            << flipperName_j << "'" << std::endl;
    }
  }

  // Set "collide_without_contact_bitmask" to the whole vehicle (default is 1u).
  // this->LoadParam(robot_model_, "collide_without_contact_bitmask",
  //                 this->collideWithoutContactBitmask, 1u);
  collideWithoutContactBitmask = 1u;

  // Initialize the Settings
  contactManager = world_model_->Physics()->GetContactManager();
  // otherwise contact manager would not publish any contacts (since we are not
  // a real contact subscriber)
  contactManager->SetNeverDropContacts(true);

  // set correct categories and collide bitmasks
  SetGeomCategories();

  // set the desired friction to tracks (override the values set in the SDF model)
  UpdateTrackSurface();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libNuBotPumbaaPlugin.so' in the gazebo_ros package)");
    return;
  }
  // Load the ROS settings
  rosnode_ = new ros::NodeHandle(robot_name_);

  // Publishers
  // omin_vision_pub_   = rosnode_->advertise<nubot_common::OminiVisionInfo>("omnivision/OmniVisionInfo",10);
  // debug_pub_ = rosnode_->advertise<std_msgs::Float64MultiArray>("debug",10);
  RobotState_pub_ = rosnode_->advertise<geometry_msgs::Pose>("nubotstate/robotstate",10);

  // Subscribers.
  //ros::SubscribeOptions so1 = ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
  //            "/gazebo/model_states", 100, boost::bind( &NuBotPumbaaGazebo::model_states_CB,this,_1),
  //            ros::VoidPtr(), &message_queue_);
  //ModelStates_sub_ = rosnode_->subscribe(so1);

  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<nubot_pumbaa_msg::PumbaaCmd>(
            "nubotcontrol/pumbaacmd", 100, boost::bind(&NuBotPumbaaGazebo::Pumbaa_Cmd_CB,this,_1),
            ros::VoidPtr(), &message_queue_);
  PumbaaCmd_sub_ = rosnode_->subscribe(so2);

  // Service Servers & clients
  // dribbleId_client_ = rosnode_->serviceClient<nubot_common::DribbleId>("/DribbleId");

#if 0
    reconfigureServer_ = new dynamic_reconfigure::Server<nubot_gazebo::NuBotPumbaaGazeboConfig>(*rosnode_);
    reconfigureServer_->setCallback(boost::bind(&NuBotPumbaaGazebo::config, this, _1, _2));
#endif

  // Custom Callback Queue Thread. Use threads to process message and service callback queue
  message_callback_queue_thread_ = boost::thread( boost::bind( &NuBotPumbaaGazebo::message_queue_thread,this ) );
  //service_callback_queue_thread_ = boost::thread( boost::bind( &NuBotPumbaaGazebo::service_queue_thread,this ) );

  // This event is broadcast every simulation iteration.
  // "ConnectWorldUpdateBegin" is replaced by "ConnectBeforePhysicsUpdate"
  // Call "DriveTracks" in "update_child" to move the robot
  // update_connection_ = event::Events::ConnectWorldUpdateBegin(
  //         boost::bind(&NuBotPumbaaGazebo::update_child, this));
  update_connection_ = event::Events::ConnectBeforePhysicsUpdate(
              boost::bind(&NuBotPumbaaGazebo::update_child, this));

  // Output info
  // ROS_INFO(" %s  id: %d  flip_cord:%d  gaussian scale: %f  rate: %f\n",
  //           robot_name_.c_str(),  AgentID_, flip_cord_, noise_scale_, noise_rate_);
}

void NuBotPumbaaGazebo::Reset()
{
  ROS_DEBUG("%s Reset() running now!", robot_name_.c_str());
  SetTrackVelocity(0., 0.);
  SetFlipPose(-flip_initial, -flip_initial, flip_initial, flip_initial);

  ModelPlugin::Reset();
}

// Transform the linear/angular speed to tracks speed, called by Track_Cmd_CB.
void NuBotPumbaaGazebo::SetBodyVelocity(
    const double _linear, const double _angular)
{
  // Compute effective linear and angular speed.
  const auto linearSpeed = ignition::math::clamp(
    _linear,
    -maxLinearSpeed,
    maxLinearSpeed);
  const auto angularSpeed = ignition::math::clamp(
    _angular,
    -maxAngularSpeed,
    maxAngularSpeed);

  // Compute track velocities using the tracked vehicle kinematics model.
  const auto leftVelocity = linearSpeed + angularSpeed *
    tracksSeparation / 2 / steeringEfficiency;
  const auto rightVelocity = linearSpeed - angularSpeed *
    tracksSeparation / 2 / steeringEfficiency;

  // Call the track velocity handler (which does the actual vehicle control).
  SetTrackVelocity(leftVelocity, rightVelocity);
}

// Set the calculated tracks speed to Gazebo simulation
void NuBotPumbaaGazebo::SetTrackVelocity(double _left, double _right)
{
  //gzmsg << "NuBotPumbaaGazebo_Plugin: SetTrackVelocity is calling" << std::endl;
  // Apply the max track velocity limit.
  const auto left = ignition::math::clamp(_left,
                                          -maxLinearSpeed,
                                          maxLinearSpeed);
  const auto right = ignition::math::clamp(_right,
                                           -maxLinearSpeed,
                                           maxLinearSpeed);
  // Call the descendant custom handler of the subclass.
  SetTrackVelocityImpl(left, right);

  // Publish the resulting track velocities to anyone who is interested.
  /*
  auto speedMsg = msgs::Vector2d();
  speedMsg.set_x(left);
  speedMsg.set_y(right);
  this->dataPtr->tracksVelocityPub->Publish(speedMsg);
  */
}

void NuBotPumbaaGazebo::SetTrackVelocityImpl(double _left,
                                             double _right)
{
  //gzmsg << "NuBotPumbaaGazebo_Plugin: SetTrackVelocityImpl is calling" << std::endl;
  trackVelocity_[Tracks::LEFT] = _left;
  trackVelocity_[Tracks::RIGHT] = _right;
}

// Set the rotate position of flippers direct to the model, called by Flip_Cmd_CB.
void NuBotPumbaaGazebo::SetFlipPose(
        double _frontleft, double _frontright, double _rearleft, double _rearright)
{
    robot_model_->GetJointController()->SetPositionTarget(
                front_left_j->GetScopedName(), _frontleft);
    robot_model_->GetJointController()->SetPositionTarget(
                front_right_j->GetScopedName(), _frontright);
    robot_model_->GetJointController()->SetPositionTarget(
                rear_left_j->GetScopedName(), _rearleft);
    robot_model_->GetJointController()->SetPositionTarget(
                rear_right_j->GetScopedName(), _rearright);
}

void NuBotPumbaaGazebo::UpdateTrackSurface()
{
  auto& gtracks = globalTracks.at(body_);
  for (auto trackSide : gtracks)
  {
    for (auto track : trackSide.second)
    {
      SetLinkMu(track);
    }
  }
}

double NuBotPumbaaGazebo::GetSteeringEfficiency()
{
  return this->steeringEfficiency;
}

double NuBotPumbaaGazebo::GetTracksSeparation()
{
  return this->tracksSeparation;
}

boost::optional<double> NuBotPumbaaGazebo::GetTrackMu()
{
  return std::stod(trackMu->GetDefaultAsString());
}

boost::optional<double> NuBotPumbaaGazebo::GetTrackMu2()
{
  return std::stod(trackMu2->GetDefaultAsString());
}

//void NuBotPumbaaGazebo::SetSteeringEfficiency(double _steeringEfficiency)
//{
//  this->steeringEfficiency = _steeringEfficiency;
//  this->world_sdf_->GetElement("steering_efficiency")
//    ->Set(_steeringEfficiency);
//}

void NuBotPumbaaGazebo::SetLinkMu(const physics::LinkPtr &_link)
{
  if (!this->GetTrackMu().is_initialized() &&
    !this->GetTrackMu2().is_initialized())
  {
    return;
  }
  for (auto const &collision : _link->GetCollisions())
    {
      auto frictionPyramid = collision->GetSurface()->FrictionPyramid();
      if (frictionPyramid == nullptr)
      {
        gzwarn << "This dynamics engine doesn't support setting mu/mu2 friction"
          " parameters. Use its dedicated friction setting mechanism to set the"
          " wheel friction." << std::endl;
        break;
      }
      if (this->GetTrackMu().is_initialized())
      {
        double mu = this->GetTrackMu().get();
        if (!ignition::math::equal(frictionPyramid->MuPrimary(), mu, 1e-6))
        {
          gzdbg << "Setting mu (friction) of link '" << _link->GetName() <<
                "' from " << frictionPyramid->MuPrimary() << " to " <<
                mu << std::endl;
        }
        frictionPyramid->SetMuPrimary(mu);
      }
      if (this->GetTrackMu2().is_initialized())
      {
        double mu2 = this->GetTrackMu2().get();
        if (!ignition::math::equal(frictionPyramid->MuSecondary(), mu2, 1e-6))
        {
          gzdbg << "Setting mu2 (friction) of link '" << _link->GetName() <<
                "' from " << frictionPyramid->MuSecondary() << " to " <<
                mu2 << std::endl;
        }
        frictionPyramid->SetMuSecondary(mu2);
      }
    }
}

void NuBotPumbaaGazebo::SetGeomCategories()
{
  auto linksToProcess = body_->GetModel()->GetLinks();
  // set ROBOT_CATEGORY to the whole body and all subparts
  physics::LinkPtr link;
  while (!linksToProcess.empty())
  {
    link = linksToProcess.back();
    linksToProcess.pop_back();

    auto childLinks = link->GetChildJointsLinks();
    linksToProcess.insert(linksToProcess.end(), childLinks.begin(),
                          childLinks.end());

    for (auto const &collision : link->GetCollisions())
    {
      collision->SetCategoryBits(ROBOT_CATEGORY);
      collision->SetCollideBits(GZ_FIXED_COLLIDE);

      GZ_ASSERT(collision->GetSurface() != nullptr,
                "Collision surface is nullptr");
      collision->GetSurface()->collideWithoutContactBitmask =
        this->collideWithoutContactBitmask;
    }
  }

  auto& gtracks = globalTracks.at(body_);
  for (auto trackSide : gtracks)
  {
    for (auto trackLink : trackSide.second)
    {
      auto bits = ROBOT_CATEGORY | BELT_CATEGORY;
      if (trackSide.first == Tracks::LEFT)
        bits |= LEFT_CATEGORY;

      for (auto const &collision : trackLink->GetCollisions())
      {
        collision->SetCategoryBits(bits);
      }
    }
  }
}

void NuBotPumbaaGazebo::DriveTracks(/*const common::UpdateInfo &_unused*/)
{
  if (this->contactManager->GetContactCount() == 0)
    return;

  /////////////////////////////////////////////
  // Calculate the desired center of rotation
  /////////////////////////////////////////////

  const auto leftBeltSpeed = -this->trackVelocity_[Tracks::LEFT];
  const auto rightBeltSpeed = -this->trackVelocity_[Tracks::RIGHT];

  // the desired linear and angular speeds (set by desired track velocities)
  const auto linearSpeed = (leftBeltSpeed + rightBeltSpeed) / 2;
  const auto angularSpeed = -(leftBeltSpeed - rightBeltSpeed) *
    this->GetSteeringEfficiency() / this->GetTracksSeparation();

  // radius of the turn the robot is doing
  const auto desiredRotationRadiusSigned =
                               (fabs(angularSpeed) < 0.1) ?
                               // is driving straight
                               dInfinity :
                               (
                                 (fabs(linearSpeed) < 0.1) ?
                                 // is rotating about a single point
                                 0 :
                                 // general movement
                                 linearSpeed / angularSpeed);

  const auto bodyPose = this->body_->WorldPose();
  const auto bodyYAxisGlobal =
    bodyPose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  const auto centerOfRotation =
    (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

  ////////////////////////////////////////////////////////////////////////
  // For each contact, compute the friction force direction and speed of
  // surface movement.
  ////////////////////////////////////////////////////////////////////////
  size_t i = 0;
  const auto contacts = this->contactManager->GetContacts();
  const auto model = this->body_->GetModel();

  for (auto contact : contacts)
  {
    // Beware! There may be invalid contacts beyond GetContactCount()...
    if (i == this->contactManager->GetContactCount())
      break;

    ++i;

    if (contact->collision1->GetSurface()->collideWithoutContact ||
      contact->collision2->GetSurface()->collideWithoutContact)
    {
      // SKip the contact point with collisions without contact joint
      continue;
    }

    if (!contact->collision1->GetLink()->GetEnabled() ||
      !contact->collision2->GetLink()->GetEnabled())
    {
      // SKip the contact point without physics engine enabled
      continue;
    }

    if (contact->collision1->IsStatic() && contact->collision2->IsStatic())
    {
      // we're not interested in static model collisions
      // (they do not have any ODE bodies).
      continue;
    }

    if (model != contact->collision1->GetLink()->GetModel() &&
        model != contact->collision2->GetLink()->GetModel())
    {
      // Verify one of the collisions' bodies is a track of this vehicle
      continue;
    }

    dBodyID body1 = dynamic_cast<physics::ODELink&>(
      *contact->collision1->GetLink()).GetODEId();
    dBodyID body2 = dynamic_cast<physics::ODELink& >(
      *contact->collision2->GetLink()).GetODEId();

    dGeomID geom1 = dynamic_cast<physics::ODECollision& >(
      *contact->collision1).GetCollisionId();
    dGeomID geom2 = dynamic_cast<physics::ODECollision& >(
      *contact->collision2).GetCollisionId();

    bool bodiesSwapped = false;
    if (body1 == 0)
    {
      std::swap(body1, body2);
      std::swap(geom1, geom2);

      // we'll take care of the normal flipping later
      bodiesSwapped = true;
    }

    // determine if track is the first or second collision element
    const bool isGeom1Track = (dGeomGetCategoryBits(geom1) & BELT_CATEGORY) > 0;
    const bool isGeom2Track = (dGeomGetCategoryBits(geom2) & BELT_CATEGORY) > 0;

    if (!isGeom1Track && !isGeom2Track)
      continue;

    // speed and geometry of the track in collision
    const auto trackGeom = (isGeom1Track ? geom1 : geom2);
    // the != means XOR here; we basically want to get the collision belonging
    // to the track, but we might have swapped the ODE bodies in between,
    // so we have to account for it
    const physics::Collision* trackCollision =
      ((isGeom1Track != bodiesSwapped) ? contact->collision1
                                       : contact->collision2);
    const dReal beltSpeed =
      (dGeomGetCategoryBits(trackGeom) & LEFT_CATEGORY) != 0 ?
      leftBeltSpeed : rightBeltSpeed;

    // remember if we've found at least one contact joint (we should!)
    bool foundContact = false;
    for (auto contactIterator = ContactIterator::begin(body1, geom1, geom2);
         contactIterator != ContactIterator::end();
         ++contactIterator)
    {
      dContact* odeContact = contactIterator.getPointer();

      // now we're sure it is a contact between our two geometries
      foundContact = true;

      const ignition::math::Vector3d contactWorldPosition(
        odeContact->geom.pos[0],
        odeContact->geom.pos[1],
        odeContact->geom.pos[2]);

      ignition::math::Vector3d contactNormal(
        odeContact->geom.normal[0],
        odeContact->geom.normal[1],
        odeContact->geom.normal[2]);

      // We always want contactNormal to point "inside" the track.
      // The dot product is 1 for co-directional vectors and -1 for
      // opposite-pointing vectors.
      // The contact can be flipped either by swapping body1 and body2 above,
      // or by having some flipped faces on collision meshes.
      const double normalToTrackCenterDot =
        contactNormal.Dot(
          trackCollision->WorldPose().Pos() - contactWorldPosition);
      if (normalToTrackCenterDot < 0)
      {
        contactNormal = -contactNormal;
      }

      // vector tangent to the belt pointing in the belt's movement direction
      auto beltDirection(contactNormal.Cross(bodyYAxisGlobal));

      if (beltSpeed > 0)
        beltDirection = -beltDirection;

      const auto frictionDirection =
        this->ComputeFrictionDirection(linearSpeed,
                                       angularSpeed,
                                       desiredRotationRadiusSigned == dInfinity,
                                       bodyPose,
                                       bodyYAxisGlobal,
                                       centerOfRotation,
                                       contactWorldPosition,
                                       contactNormal,
                                       beltDirection);

      odeContact->fdir1[0] = frictionDirection.X();
      odeContact->fdir1[1] = frictionDirection.Y();
      odeContact->fdir1[2] = frictionDirection.Z();

      // use friction direction and motion1 to simulate the track movement
      odeContact->surface.mode |= dContactFDir1 | dContactMotion1;

      odeContact->surface.motion1 = this->ComputeSurfaceMotion(
        beltSpeed, beltDirection, frictionDirection);
    }

    if (!foundContact)
    {
      gzwarn << "No ODE contact joint found for contact " <<
             contact->DebugString() << std::endl;
      continue;
    }
  }
}

ignition::math::Vector3d NuBotPumbaaGazebo::ComputeFrictionDirection(
  const double _linearSpeed, const double _angularSpeed,
  const bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
  const ignition::math::Vector3d &_bodyYAxisGlobal,
  const ignition::math::Vector3d &_centerOfRotation,
  const ignition::math::Vector3d &_contactWorldPosition,
  const ignition::math::Vector3d &_contactNormal,
  const ignition::math::Vector3d &_beltDirection) const
{
  ignition::math::Vector3d frictionDirection;

  if (!_drivingStraight)
  {
    // non-straight drive

    // vector pointing from the center of rotation to the contact point
    const auto COR2Contact =
      (_contactWorldPosition - _centerOfRotation).Normalize();

    // the friction force should be perpendicular to COR2Contact
    frictionDirection = _contactNormal.Cross(COR2Contact);

    // position of the contact point relative to vehicle body
    const auto contactInVehiclePos =
        _bodyPose.Rot().RotateVectorReverse(
          _contactWorldPosition - _bodyPose.Pos());

    const int linearSpeedSignum =
        (fabs(_linearSpeed) > 0.1) ? ignition::math::signum(_linearSpeed) : 1;

    // contactInVehiclePos.Dot(ignition::math::Vector3d(1, 0, 0)) > 0 means
    // the contact is "in front" of the line on which COR moves
    if ((ignition::math::signum(_angularSpeed) *
      ignition::math::signum(_bodyYAxisGlobal.Dot(frictionDirection))) !=
      (linearSpeedSignum *
        ignition::math::signum(contactInVehiclePos.Dot(
          ignition::math::Vector3d(1, 0, 0)))))
    {
      frictionDirection = -frictionDirection;
    }

    if (_linearSpeed < 0)
      frictionDirection = - frictionDirection;
  }
  else
  {
    // straight drive
    frictionDirection = _contactNormal.Cross(_bodyYAxisGlobal);

    if (frictionDirection.Dot(_beltDirection) < 0)
      frictionDirection = -frictionDirection;
  }

  return frictionDirection;
}

double NuBotPumbaaGazebo::ComputeSurfaceMotion(const double _beltSpeed,
    const ignition::math::Vector3d &_beltDirection,
    const ignition::math::Vector3d &_frictionDirection) const
{
  // the dot product <beltDirection,fdir1> is the cosine of the angle they
  // form (because both are unit vectors)
  // the motion is in the opposite direction than the desired motion of the body
  return -_beltDirection.Dot(_frictionDirection) * fabs(_beltSpeed);
}

NuBotPumbaaGazebo::ContactIterator
NuBotPumbaaGazebo::ContactIterator::operator++()
{
  // initialized && null contact means we've reached the end of the iterator
  if (this->initialized && this->currentContact == nullptr)
  {
    return *this;
  }

  // I haven't found a nice way to get ODE ID of the collision joint,
  // so we need to iterate over all joints connecting the two colliding
  // bodies and try to find the one we're interested in.
  // This should not be a performance issue, since bodies connected by other
  // joint types do not collide by default.

  // remember if we've found at least one contact joint (we should!)
  bool found = false;
  for (; this->jointIndex < static_cast<size_t>(dBodyGetNumJoints(this->body));
         this->jointIndex++)
  {
    const auto joint = dBodyGetJoint(this->body,
                                     static_cast<int>(this->jointIndex));

    // only interested in contact joints
    if (dJointGetType(joint) != dJointTypeContact)
      continue;

    // HACK here we unfortunately have to access private ODE data
    // It must really be static_cast here; if dynamic_cast is used, the runtime
    // cannot find RTTI for dxJointContact and its predecessors.
    dContact* odeContact = &(static_cast<dxJointContact*>(joint)->contact);

    if (!(
            odeContact->geom.g1 == this->geom1 &&
            odeContact->geom.g2 == this->geom2)
        &&
        !(
            odeContact->geom.g1 == this->geom2 &&
            odeContact->geom.g2 == this->geom1))
    {
      // not a contact between our two geometries
      continue;
    }

    // we found a contact we're interested in

    found = true;
    this->initialized = true;

    // we can be pretty sure the contact instance won't get deleted until this
    // code finishes, since we are in a pause between contact generation and
    // physics update
    this->currentContact = odeContact;

    // needed since we break out of the for-loop
    this->jointIndex++;
    break;
  }

  if (!found)
  {
    // we've reached the end of the iterator
    this->currentContact = nullptr;
    this->initialized = true;
  }

  this->initialized = true;
  return *this;
}

NuBotPumbaaGazebo::ContactIterator::ContactIterator()
    : currentContact(nullptr), jointIndex(0), body(nullptr), geom1(nullptr),
      geom2(nullptr), initialized(false)
{
}

NuBotPumbaaGazebo::ContactIterator::ContactIterator(
    bool _initialized) : currentContact(nullptr), jointIndex(0), body(nullptr),
                         geom1(nullptr), geom2(nullptr),
                         initialized(_initialized)
{
}

NuBotPumbaaGazebo::ContactIterator::ContactIterator(
    const NuBotPumbaaGazebo::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;
}

NuBotPumbaaGazebo::ContactIterator::ContactIterator(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2) :
    currentContact(nullptr), jointIndex(0), body(_body),
    geom1(_geom1), geom2(_geom2), initialized(false)
{
}

NuBotPumbaaGazebo::ContactIterator
NuBotPumbaaGazebo::ContactIterator::begin(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2)
{
  return ContactIterator(_body, _geom1, _geom2);
}

NuBotPumbaaGazebo::ContactIterator
NuBotPumbaaGazebo::ContactIterator::end()
{
  return ContactIterator(true);
}

bool NuBotPumbaaGazebo::ContactIterator::operator==(
    const NuBotPumbaaGazebo::ContactIterator &_rhs)
{
  if (this->currentContact == nullptr && !this->initialized)
    ++(*this);

  return this->currentContact == _rhs.currentContact &&
         this->initialized == _rhs.initialized;
}

NuBotPumbaaGazebo::ContactIterator&
NuBotPumbaaGazebo::ContactIterator::operator=(
    const NuBotPumbaaGazebo::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;

  return *this;
}

NuBotPumbaaGazebo::ContactIterator
NuBotPumbaaGazebo::ContactIterator::operator++(int /*_unused*/)
{
  ContactIterator i = *this;
  ++(*this);
  return i;
}

NuBotPumbaaGazebo::ContactIterator::reference
NuBotPumbaaGazebo::ContactIterator::operator*()
{
  if (!this->initialized)
    ++(*this);

  return *this->currentContact;
}

NuBotPumbaaGazebo::ContactIterator::pointer
NuBotPumbaaGazebo::ContactIterator::operator->()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

NuBotPumbaaGazebo::ContactIterator::pointer
NuBotPumbaaGazebo::ContactIterator::getPointer()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

bool NuBotPumbaaGazebo::ContactIterator::operator!=(
    const NuBotPumbaaGazebo::ContactIterator &_rhs)
{
  return !NuBotPumbaaGazebo::ContactIterator::operator==(_rhs);
}


void NuBotPumbaaGazebo::update_child()
{
  msgCB_lock_.lock(); // lock access to fields that are used in ROS message callbacks
  //srvCB_lock_.lock();
  /* delay in model_states messages publishing
   * so after receiving model_states message, then nubot moves. */
  //if(update_model_info())
  //{
      /********** EDIT BEGINS **********/

      //nubot_be_control();
      //nubot_test();
  this->DriveTracks();
  message_publish();
  //this->SetFlipPose(0,front_right_j_speed,0,0);
  //gzmsg << "NuBotPumbaaGazebo_Plugin: update_child is running" << std::endl;

      /**********  EDIT ENDS  **********/
  //}
  //srvCB_lock_.unlock();
  msgCB_lock_.unlock();
}

void NuBotPumbaaGazebo::message_publish(void)
{
  geometry_msgs::Pose RobotPose;
  RobotPose.position.x = body_->WorldPose().Pos().X();
  RobotPose.position.y = body_->WorldPose().Pos().Y();
  RobotPose.position.z = body_->WorldPose().Pos().Z();

  RobotPose.orientation.x = body_->WorldPose().Rot().X();
  RobotPose.orientation.y = body_->WorldPose().Rot().Y();
  RobotPose.orientation.z = body_->WorldPose().Rot().Z();
  RobotPose.orientation.w = body_->WorldPose().Rot().W();
  RobotState_pub_.publish(RobotPose);
}

void NuBotPumbaaGazebo::Pumbaa_Cmd_CB(const nubot_pumbaa_msg::PumbaaCmd::ConstPtr &_msg)
{
  gzmsg << "NuBotPumbaaGazebo_Plugin: Pumbaa_Cmd_CB " << "ContactCount=" <<
        std::to_string(contactManager->GetContactCount()) << std::endl;
//  gzmsg << "NuBotPumbaaGazebo_Plugin: Track_Cmd_CB body_->WorldPose Pos.X:"
//        << std::to_string(body_->WorldPose().Pos().X()) << std::endl;
//  gzmsg << "NuBotPumbaaGazebo_Plugin: Track_Cmd_CB body_->WorldPose Pos.Y:"
//        << std::to_string(body_->WorldPose().Pos().Y()) << std::endl;
//  gzmsg << "NuBotPumbaaGazebo_Plugin: Track_Cmd_CB body_->WorldPose Pos.Z:"
//        << std::to_string(body_->WorldPose().Pos().Z()) << std::endl;
  msgCB_lock_.lock();
  double MsglinearSpeed = _msg->vel_linear;
  double MsgangularSpeed = _msg->vel_angular;

  double MsgFL = _msg->front_left;
  double MsgFR = _msg->front_right;
  double MsgRL = _msg->rear_left;
  double MsgRR = _msg->rear_right;

  SetBodyVelocity(MsglinearSpeed,MsgangularSpeed);
  SetFlipPose(-MsgFL,-MsgFR,MsgRL,MsgRR);
  msgCB_lock_.unlock();
}

void NuBotPumbaaGazebo::message_queue_thread()
{
  static const double timeout = 0.01;
  while (rosnode_->ok())
  {
    // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
    // pushes it back onto the queue. This version includes a timeout which lets you specify
    // the amount of time to wait for a callback to be available before returning.
    message_queue_.callAvailable(ros::WallDuration(timeout));
    // gzmsg << "NuBotPumbaaGazebo_Plugin: message_queue_thread is running" << std::endl;
  }
}
