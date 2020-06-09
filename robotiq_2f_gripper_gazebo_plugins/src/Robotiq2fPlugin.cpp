/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
    This file has been modified from the original, by Devon Ash and Kenneth Bogert
*/ 

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <ignition/math.hh>
#include <dynamic_reconfigure/server.h>
#include <robotiq_2f_gripper_gazebo_plugins/Robotiq2fGripperConfig.h>

/*

Due to necessity, I had to change the PID.hh file's definition from private members to public to allow public access of its members. They're private in 1.9 and getter functions aren't implemented until gazebo 3.0. I thought that was silly, and so I hacked around it. The functions directly access the private members by necessity. It can only change if Gazebo patches 1.9 and 2.2 to include getters for it.

I'm not sure exactly where the dependency chain includes PID.hh for the first time, so I've encapsulated all of the gazebo includes. Not pretty, but it works. If you're reading this and know of a better soln', feel free to change it.

*/
#define private public 
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <robotiq_2f_gripper_gazebo_plugins/Robotiq2fPlugin.h>
#undef private

// Default topic names initialization.
const std::string Robotiq2fPlugin::DefaultTopicCommand  =
  "/gripper/output";
const std::string Robotiq2fPlugin::DefaultTopicState    =
  "/gripper/input";

////////////////////////////////////////////////////////////////////////////////
Robotiq2fPlugin::Robotiq2fPlugin()
{
  // PID default parameters.
  for (int i = 0; i < this->NumJoints; ++i)
  {
    this->posePID[i].Init(1, 0.01, 0.0, this->MaxVelocity, -this->MaxVelocity, this->MaxVelocity, -this->MaxVelocity);
    this->posePID[i].SetCmd(0.0);
    this->velPID[i].Init(1, 0.01, 0.0, 220, -220.0, 220.0, -220.0);
    this->velPID[i].SetCmd(0.0);
  }

  // Default hand state: Disabled.
  this->handState = Disabled;
}

////////////////////////////////////////////////////////////////////////////////
Robotiq2fPlugin::~Robotiq2fPlugin()
{
#if GAZEBO_MAJOR_VERSION < 9
	gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#else
	this->updateConnection.reset();
#endif
  this->rosNode->shutdown();
  this->dynReconRosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->sdf = _sdf;


  // Load the vector of all joints.
  if (!this->FindJoints())
    return;

  gzlog << "Prior to iterating.." << std::endl;
  // Initialize joint state vector.
  this->jointStates.name.resize(this->jointNames.size()-1);
  this->jointStates.position.resize(this->jointNames.size()-1);
  this->jointStates.velocity.resize(this->jointNames.size()-1);
  this->jointStates.effort.resize(this->jointNames.size()-1);
  gzlog << "About to iterate things.." << std::endl;
  for (size_t i = 1; i < this->jointNames.size(); ++i)
  {
    this->jointStates.name[i-1] = this->jointNames[i];
    this->jointStates.position[i-1] = 0;
    this->jointStates.velocity[i-1] = 0;
    this->jointStates.effort[i-1] = 0;
  }
  gzlog << "Initialized the joint state vector" << std::endl;

  // Default ROS topic names.
  std::string controlTopicName = this->DefaultTopicCommand;
  std::string stateTopicName   = this->DefaultTopicState;
  gzlog << "Using control topic " << controlTopicName << std::endl;

  for (int i = 0; i < this->NumJoints; ++i)
  {

    // Set the PID effort limits.
    this->velPID[i].SetCmdMin(-this->joints[i]->GetEffortLimit(0));
    this->velPID[i].SetCmdMax(this->joints[i]->GetEffortLimit(0));

  }

  // Overload the ROS topics for the hand if they are available.
  if (this->sdf->HasElement("topic_command"))
    controlTopicName = this->sdf->Get<std::string>("topic_command");

  if (this->sdf->HasElement("topic_state"))
    stateTopicName = this->sdf->Get<std::string>("topic_state");

  // Initialize ROS.
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized. Try starting gazebo with ROS plugin:\n"
          << " gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // Create a ROS node.
  this->rosNode.reset(new ros::NodeHandle(""));

  // Publish multi queue.
  this->pmq.startServiceThread();

  // Broadcasts state.
  this->pubHandleStateQueue = this->pmq.addPub<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>();
  this->pubHandleState = this->rosNode->advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>(
    stateTopicName, 100, true);

  std::string jointStateTopicName = std::string("joint_states");
  if (this->sdf->HasElement("joint_states"))
    jointStateTopicName = this->sdf->Get<std::string>("joint_states");


  // Broadcast joint state.
  this->pubJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(jointStateTopicName, 10);

  // Subscribe to user published handle control commands.
  ros::SubscribeOptions handleCommandSo =
    ros::SubscribeOptions::create<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>(
      controlTopicName, 100,
      boost::bind(&Robotiq2fPlugin::SetHandleCommand, this, _1),
      ros::VoidPtr(), &this->rosQueue);

  // Enable TCP_NODELAY since TCP causes bursty communication with high jitter.
  handleCommandSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  this->subHandleCommand = this->rosNode->subscribe(handleCommandSo);

  // Controller time control.
#if GAZEBO_MAJOR_VERSION >= 9
  this->lastControllerUpdateTime = this->world->SimTime();
#else
  this->lastControllerUpdateTime = this->world->GetSimTime();
#endif

  // Start callback queue.
  this->callbackQueueThread =
    boost::thread(boost::bind(&Robotiq2fPlugin::RosQueueThread, this));

  // Connect to gazebo world update.
  this->updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Robotiq2fPlugin::UpdateStates, this));

  // Create loop joint links on the gripper
  // Since Gazebo doesn't support joints setup in a loop correctly
  // Based upon pegasus_gazebo_plugins https://github.com/wojiaojiao/pegasus_gazebo_plugins


  sdf::ElementPtr elem = _sdf->GetElement("loop_joint");

  while (elem) {

    if(!elem->HasAttribute("name"))
    {
      ROS_ERROR("No name attribute present. Cannot setup loop joint.");
      return;
    }
    std::string joint_name_ = elem->GetAttribute("name")->GetAsString();


    if(!elem->HasAttribute("child"))
    {
      ROS_ERROR("No child attribute present. Cannot setup loop joint.");
      return;
    }

    std::string child_name_ = elem->GetAttribute("child")->GetAsString();

    if(!elem->HasAttribute("parent"))
    {
      ROS_ERROR("No parent element present. Cannot setup loop joint.");
      return;
    }

    std::string parent_name_ = elem->GetAttribute("parent")->GetAsString();

    gazebo::physics::LinkPtr child_ = this->model->GetLink(child_name_);
    if(!child_)
    {
      ROS_ERROR("No Link named %s. Cannot setup loop joint.", child_name_.c_str());
      return;
    }

    gazebo::physics::LinkPtr parent_ = this->model->GetLink(parent_name_);
    if(!parent_)
    {
      ROS_ERROR("No Link named %s. Cannot setup loop joint.", parent_name_.c_str());
      return;
    }

    #if GAZEBO_MAJOR_VERSION >= 9
    gazebo::physics::JointPtr joint = this->world->Physics()->CreateJoint("fixed");
    joint->SetName(joint_name_);
    ignition::math::Pose3d jointOrigin(0.00,0.018,-0.006,0.00,-0.00,0.00);
    joint->Load(parent_,child_,jointOrigin);
    joint->Init();
    #else
    gazebo::physics::JointPtr joint = this->world->GetPhysicsEngine()->CreateJoint("fixed");
    joint->SetName(joint_name_);
    gazebo::math::Pose jointOrigin(0.00,0.018,-0.006,0.00,-0.00,0.00);
    joint->Load(parent_,child_,jointOrigin);
    joint->Init();
    #endif

    elem = elem->GetNextElement("loop_joint");

  }

  // Setup Dynamic Reconfigure
  dynamic_reconfigure::Server<robotiq_2f_gripper_gazebo_plugins::Robotiq2fGripperConfig>::CallbackType f;

  f = boost::bind(&Robotiq2fPlugin::dynamic_reconfigure_callback, this, _1, _2);

  this->dynReconRosNode.reset(new ros::NodeHandle("/gazebo_ros_control/pid_gains/finger_joint"));
  this->srv_.reset(new dynamic_reconfigure::Server<robotiq_2f_gripper_gazebo_plugins::Robotiq2fGripperConfig>(*(this->dynReconRosNode)));
  this->srv_->setCallback(f);

  // Initialize gripper

  handleCommand.rACT = 1;
  handleCommand.rATR = 0;
  handleCommand.rGTO = 1;
  handleCommand.rPR = 0;
  handleCommand.rSP = 255;


  // Log information.
  gzlog << "Robotiq2fPlugin loaded" << std::endl;
  for (int i = 0; i < this->NumJoints; ++i)
  {
    gzlog << "Position PID parameters for joint ["
          << this->joints[i]->GetName() << "]:"     << std::endl
          << "\tKP: "     << this->posePID[i].pGain  << std::endl
          << "\tKI: "     << this->posePID[i].iGain  << std::endl
          << "\tKD: "     << this->posePID[i].dGain  << std::endl
          << "\tIMin: "   << this->posePID[i].iMin   << std::endl
          << "\tIMax: "   << this->posePID[i].iMax   << std::endl
          << "\tCmdMin: " << this->posePID[i].cmdMin << std::endl
          << "\tCmdMax: " << this->posePID[i].cmdMax << std::endl
          << std::endl;
  }
  gzlog << "Topic for sending hand commands: ["   << controlTopicName
        << "]\nTopic for receiving hand state: [" << stateTopicName
        << "]" << std::endl;
}



////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::dynamic_reconfigure_callback(robotiq_2f_gripper_gazebo_plugins::Robotiq2fGripperConfig &config, uint32_t level) {

    posePID[0].SetPGain(config.finger_p);
    posePID[0].SetIGain(config.finger_i);
    posePID[0].SetDGain(config.finger_d);
    velPID[0].SetPGain(config.finger_velocity_p);
    velPID[0].SetIGain(config.finger_velocity_i);
    velPID[0].SetDGain(config.finger_velocity_d);
}

////////////////////////////////////////////////////////////////////////////////
bool Robotiq2fPlugin::VerifyField(const std::string &_label, int _min,
  int _max, int _v)
{
  if (_v < _min || _v > _max)
  {
    std::cerr << "Illegal " << _label << " value: [" << _v << "]. The correct "
              << "range is [" << _min << "," << _max << "]" << std::endl;
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Robotiq2fPlugin::VerifyCommand(
    const robotiq_2f_gripper_control::Robotiq2FGripper_robot_output::ConstPtr &_command)
{
  return this->VerifyField("rACT", 0, 1,   _command->rACT) &&
         this->VerifyField("rGTO", 0, 1,   _command->rGTO) &&
         this->VerifyField("rATR", 0, 1,   _command->rATR) &&
         this->VerifyField("rPR", 0, 255, _command->rPR) &&
         this->VerifyField("rSP", 0, 255, _command->rSP) &&
         this->VerifyField("rFR", 0, 255, _command->rFR);
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::SetHandleCommand(
    const robotiq_2f_gripper_control::Robotiq2FGripper_robot_output::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Sanity check.
  if (!this->VerifyCommand(_msg))
  {
    std::cerr << "Ignoring command" << std::endl;
    return;
  }

  this->prevCommand = this->handleCommand;

  // Update handleCommand.
  this->handleCommand = *_msg;
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::ReleaseHand()
{
  // Open the fingers.
  this->handleCommand.rPR = 0;

  // Half speed.
  this->handleCommand.rSP = 127;
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::StopHand()
{
  // Set the target positions to the current ones.
  this->handleCommand.rPR = this->handleState.gPR;
}

////////////////////////////////////////////////////////////////////////////////
bool Robotiq2fPlugin::IsHandFullyOpen()
{
  bool fingersOpen = true;

  // The hand will be fully open when all the fingers are within 'tolerance'
  // from their lower limits.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Angle tolerance;
  tolerance.Degree(1.0);
#else
  gazebo::math::Angle tolerance;
  tolerance.SetFromDegree(1.0);
#endif

  for (int i = 0; i < this->NumJoints; ++i)
  {
    fingersOpen = fingersOpen &&
#if GAZEBO_MAJOR_VERSION >= 9
      (this->joints[i]->Position(0) < (this->joints[i]->LowerLimit(0) + tolerance()));
#else
      (this->joints[i]->GetAngle(0) < (this->joints[i]->GetLowerLimit(0) + tolerance));
#endif
  }

  return fingersOpen;
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::UpdateStates()
{
  boost::mutex::scoped_lock lock(this->controlMutex);
#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::common::Time curTime = this->world->SimTime();
#else
  gazebo::common::Time curTime = this->world->GetSimTime();
#endif

  // Step 1: State transitions.
  if (curTime > this->lastControllerUpdateTime)
  {
    this->userHandleCommand = this->handleCommand;

    // Deactivate gripper.
    if (this->handleCommand.rACT == 0)
    {
      this->handState = Disabled;
    }
    // Emergency auto-release.
    else if (this->handleCommand.rATR == 1)
    {
      this->handState = Emergency;
    }
    else
    {
      this->handState = GoTo;
    }

    // Step 2: Actions in each state.
    switch (this->handState)
    {
      case Disabled:
        break;

      case Emergency:
        // Open the hand.
        if (this->IsHandFullyOpen())
          this->StopHand();
        else
          this->ReleaseHand();
        break;

      case GoTo:
        if (this->handleCommand.rGTO == 0)
        {
          // "Stop" action.
          this->StopHand();
        }
        break;

      default:
        std::cerr << "Unrecognized state [" << this->handState << "]"
                  << std::endl;
    }

    // Update the hand controller.
    this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());

    // Gather robot state data and publish them.
    this->GetAndPublishHandleState();

    // Publish joint states.
    this->GetAndPublishJointState(curTime);


    this->lastControllerUpdateTime = curTime;
  }
}

////////////////////////////////////////////////////////////////////////////////
uint8_t Robotiq2fPlugin::GetObjectDetection(
  const gazebo::physics::JointPtr &_joint, int _index, uint8_t _rPR,
  uint8_t _prevrPR)
{
  // Check finger's speed.
  bool isMoving = _joint->GetVelocity(0) > this->VelTolerance;

  // Check if the finger reached its target positions. We look at the error in
  // the position PID to decide if reached the target.
  double pe, ie, de;
  this->posePID[_index].GetErrors(pe, ie, de);
  bool reachPosition = pe < this->PoseTolerance;

  if (isMoving)
  {
    // Finger is in motion.
    return 0;
  }
  else
  {
    if (reachPosition)
    {
      // Finger is at the requestedPosition.
      return 3;
    }
    else if (_rPR - _prevrPR > 0)
    {
      // Finger has stopped due to a contact while closing.
      return 2;
    }
    else
    {
      // Finger has stopped due to a contact while opening.
      return 1;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
uint8_t Robotiq2fPlugin::GetCurrentPosition(
  const gazebo::physics::JointPtr &_joint)
{
  // Full range of motion.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Angle range = _joint->UpperLimit(0) - _joint->LowerLimit(0);
#else
  gazebo::math::Angle range = _joint->GetUpperLimit(0) - _joint->GetLowerLimit(0);
#endif

  // Angle relative to the lower limit.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Angle relAngle = _joint->Position(0) - _joint->LowerLimit(0);
#else
  gazebo::math::Angle relAngle = _joint->GetAngle(0) - _joint->GetLowerLimit(0);
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  return static_cast<uint8_t>(round(255.0 * relAngle() / range()));
#else
  return static_cast<uint8_t>(round(255.0 * relAngle.Radian() / range.Radian()));
#endif
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::GetAndPublishHandleState()
{

  // gACT. Initialization status.
  this->handleState.gACT = this->userHandleCommand.rACT;

  // gGTO. Action status.
  this->handleState.gGTO = this->userHandleCommand.rGTO;

  // Check fingers' speed.
  bool isMoving = this->joints[0]->GetVelocity(0) > this->VelTolerance;

  // Check if the fingers reached their target positions.
  double pe, ie, de;
  this->posePID[2].GetErrors(pe, ie, de);
  bool reachPosition = pe < this->PoseTolerance;

  // gSTA. Gripper status.
  if (this->userHandleCommand.rACT == 0 || this->userHandleCommand.rATR == 1)
  {
    // Gripper is reset or in automatic release.
    this->handleState.gSTA = 0;
  }
  else
  {
    this->handleState.gSTA = 3;
  }

  // gOBJ. Finger object detection.
  this->handleState.gOBJ = this->GetObjectDetection(this->joints[0], 0,
    this->handleCommand.rPR, this->prevCommand.rPR);

/*
#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::common::Time curTime = this->world->SimTime();
#else
  gazebo::common::Time curTime = this->world->GetSimTime();
#endif

  gazebo::common::Time oneSecond = gazebo::common::Time(1, 0);
*/

  // gFLT. Fault status.
  if (this->handState == Disabled)
    this->handleState.gFLT = 7;
  else if (this->handState == Emergency) {
    if (this->IsHandFullyOpen())
        this->handleState.gFLT = 15;
    else
        this->handleState.gFLT = 11;
//  else if (this->handleCommand.header.stamp < curTime - oneSecond)
//    this->handleState.gFLT = 9;
  }
  else
    this->handleState.gFLT = 0;

  // gPRA. Echo of requested position for finger.
  this->handleState.gPR = this->userHandleCommand.rPR;
  // gPOA. Finger position [0-255].
  this->handleState.gPO = this->GetCurrentPosition(this->joints[0]);

//  this->handleState.gCU = (int)(this->joints[0]->GetForce(0u) * 255);
  this->handleState.gCU = 0; // not implemented

  // Publish robot states.
  this->pubHandleStateQueue->push(this->handleState, this->pubHandleState);
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::GetAndPublishJointState(
                                           const gazebo::common::Time &_curTime)
{
  this->jointStates.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);
  for (size_t i = 1; i < this->joints.size(); ++i)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    this->jointStates.position[i-1] = this->joints[i]->Position(0);
#else
    this->jointStates.position[i-1] = this->joints[i]->GetAngle(0).Radian();
#endif
    this->jointStates.velocity[i-1] = this->joints[i]->GetVelocity(0);
    // better to use GetForceTorque dot joint axis
    this->jointStates.effort[i-1] = this->joints[i]->GetForce(0u);
  }
  this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::UpdatePIDControl(double _dt)
{
  if (this->handState == Disabled)
  {
    for (int i = 0; i < this->NumJoints; ++i)
      this->joints[i]->SetForce(0, 0.0);

    return;
  }

  for (int i = 0; i < this->NumJoints; ++i)
  {
    double targetPose = 0.0;
    double maxSpeed = this->MaxVelocity * this->handleCommand.rSP / 255.0;

#if GAZEBO_MAJOR_VERSION >= 9
    targetPose = this->joints[i]->LowerLimit(0) +
      (this->joints[i]->UpperLimit(0) -
       this->joints[i]->LowerLimit(0))
#else
    targetPose = this->joints[i]->GetLowerLimit(0).Radian() +
      (this->joints[i]->GetUpperLimit(0).Radian() -
       this->joints[i]->GetLowerLimit(0).Radian())
#endif
      * this->handleCommand.rPR / 255.0;

    // Get the current pose.
#if GAZEBO_MAJOR_VERSION >= 9
    double currentPose = this->joints[i]->Position(0);
#else
    double currentPose = this->joints[i]->GetAngle(0).Radian();
#endif

    // Position error.
    double poseError = currentPose - targetPose;

    // Update the position PID.
    #if GAZEBO_MAJOR_VERSION >= 9
    double targetSpeed = ignition::math::clamp(this->posePID[i].Update(poseError, _dt), -maxSpeed, maxSpeed);
	#else
	double targetSpeed = gazebo::math::clamp(this->posePID[i].Update(poseError, _dt), -maxSpeed, maxSpeed);
	#endif
    double velocityError = this->joints[i]->GetVelocity(0) - targetSpeed;

    double torque = this->velPID[i].Update(velocityError, _dt);
 
//    ROS_INFO("Speed %f VelocityErr %f Torque %f", targetSpeed, velocityError, torque);

    // Apply the PID command.
    this->joints[i]->SetForce(0, torque);
  }
}

////////////////////////////////////////////////////////////////////////////////
bool Robotiq2fPlugin::GetAndPushBackJoint(const std::string& _jointName,
                                            gazebo::physics::Joint_V& _joints)
{
  gazebo::physics::JointPtr joint = this->model->GetJoint(_jointName);

  if (!joint)
  {
    gzerr << "Failed to find joint [" << _jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  _joints.push_back(joint);
  gzlog << "Robotiq2fPlugin found joint [" << _jointName << "]" << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Robotiq2fPlugin::FindJoints()
{
  // Load up the joints we expect to use, finger by finger.
  gazebo::physics::JointPtr joint;

  // finger_joint (actuated).
  if (!this->GetAndPushBackJoint("finger_joint", this->joints))
    return false;
  this->jointNames.push_back("finger_joint");



  std::string joint_name = "left_inner_knuckle_dummy_joint";
  if (!this->GetAndPushBackJoint(joint_name, this->joints))
    return false;
  this->jointNames.push_back(joint_name);

  joint_name = "right_inner_knuckle_dummy_joint";
  if (!this->GetAndPushBackJoint(joint_name, this->joints))
    return false;
  this->jointNames.push_back(joint_name);


  gzlog << "Robotiq2fPlugin found all joints for gripper" << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void Robotiq2fPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(Robotiq2fPlugin)
