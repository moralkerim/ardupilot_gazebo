/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "GimbalSmall2dPlugin.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

/// \brief Private data class
class gazebo::GimbalSmall2dPluginPrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void TiltOnStringMsg(ConstGzStringPtr &_msg);
  public: void PanOnStringMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr tiltSub;
  public: transport::SubscriberPtr rollSub;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for tilting the gimbal
  public: physics::JointPtr tiltJoint;

  /// \brief Joint for rolling the gimbal
  public: physics::JointPtr rollJoint;

  /// \brief Command that updates the gimbal tilt angle
  public: double tiltSetpoint = -IGN_PI_2;
  public: double rollSetpoint = 0.0;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief PID controller for the gimbal
  public: common::PID pid;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;
};

/////////////////////////////////////////////////
GimbalSmall2dPlugin::GimbalSmall2dPlugin()
  : dataPtr(new GimbalSmall2dPluginPrivate)
{
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  std::string tiltJointName = "tilt_joint";
  if (_sdf->HasElement("tilt_joint"))
  {
    tiltJointName = _sdf->Get<std::string>("tilt_joint");
  }
  this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(tiltJointName);

  std::string rollJointName = "roll_joint";
  if (_sdf->HasElement("roll_joint"))
  {
    rollJointName = _sdf->Get<std::string>("roll_joint");
  }
  this->dataPtr->rollJoint = this->dataPtr->model->GetJoint(rollJointName);

  std::vector<double> PID = {2, 0, 0};
  if (_sdf->HasElement("pid"))
  {
    std::string PID_string = _sdf->Get<std::string>("pid");
    std::stringstream ss(PID_string);

    ss >> PID[0];
    ss >> PID[1];
    ss >> PID[2];
  }

  this->dataPtr->pid.Init(PID[0], PID[1], PID[2], 0, 0, 1.0, -1.0);

  if (!this->dataPtr->tiltJoint)
  {
    std::string scopedJointName = _model->GetScopedName() + "::" + tiltJointName;
    gzwarn << "joint [" << tiltJointName
           << "] not found, trying again with scoped joint name ["
           << scopedJointName << "]\n";
    this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->tiltJoint)
  {
    gzerr << "GimbalSmall2dPlugin::Load ERROR! Can't get joint '"
          << tiltJointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  #if GAZEBO_MAJOR_VERSION >= 9
    this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());
    this->dataPtr->lastUpdateTime = this->dataPtr->model->GetWorld()->SimTime();
  #else
    this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->GetName());
    this->dataPtr->lastUpdateTime = this->dataPtr->model->GetWorld()->GetSimTime();
  #endif

  std::string topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_tilt_cmd";
  this->dataPtr->tiltSub = this->dataPtr->node->Subscribe(topic,
      &GimbalSmall2dPluginPrivate::TiltOnStringMsg, this->dataPtr.get());

  topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_roll_cmd";
  this->dataPtr->rollSub = this->dataPtr->node->Subscribe(topic,
      &GimbalSmall2dPluginPrivate::PanOnStringMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_tilt_status";

  this->dataPtr->pub =
    this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginPrivate::TiltOnStringMsg(ConstGzStringPtr &_msg)
{
  this->tiltSetpoint = atof(_msg->data().c_str());
}

void GimbalSmall2dPluginPrivate::PanOnStringMsg(ConstGzStringPtr &_msg)
{
  this->rollSetpoint = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint)
    return;

  #if GAZEBO_MAJOR_VERSION >= 9
    double tiltAngle = this->dataPtr->tiltJoint->Position(0);
    double rollAngle = this->dataPtr->rollJoint->Position(0);
    common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  #else
    double tiltAngle = this->dataPtr->tiltJoint->GetAngle(0).Radian();
    double rollAngle = this->dataPtr->rollJoint->GetAngle(0).Radian();
    common::Time time = this->dataPtr->model->GetWorld()->GetSimTime();
  #endif

  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  else if (time > this->dataPtr->lastUpdateTime)
  {
    double dt = (this->dataPtr->lastUpdateTime - time).Double();

    double tiltError = tiltAngle - this->dataPtr->tiltSetpoint;
    double rollError = rollAngle - this->dataPtr->rollSetpoint;

    double force = this->dataPtr->pid.Update(tiltError, dt);
    this->dataPtr->tiltJoint->SetForce(0, force);

    force = this->dataPtr->pid.Update(rollError, dt);
    this->dataPtr->rollJoint->SetForce(0, force);

    this->dataPtr->lastUpdateTime = time;
  }

  static int i = 1000;
  if (++i > 100)
  {
    i = 0;
    std::stringstream ss;
    ss << "Tilt: " << tiltAngle << "Roll: " << rollAngle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->dataPtr->pub->Publish(m);
  }
}
