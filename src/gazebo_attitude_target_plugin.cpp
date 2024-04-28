/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 /**
  * @brief Vision Plugin
  *
  * This plugin simulates vehicle attitude with respect to a target
  *
  * @author Elia Tarasov <elias.tarasov@gmail.com>
  */

#include <gazebo_attitude_target_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(AttitudeTargetPlugin)

AttitudeTargetPlugin::AttitudeTargetPlugin() : ModelPlugin()
{
}

AttitudeTargetPlugin::~AttitudeTargetPlugin()
{
  _updateConnection->~Connection();
}

void AttitudeTargetPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  _namespace.clear();
  if (sdf->HasElement("robotNamespace")) {
    _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_attitude_target_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("posCamXYZ")) {
    _pose_cam_XYZ = sdf->Get<ignition::math::Vector3d>("posCamXYZ");
  } else {
    _pose_cam_XYZ = ignition::math::Vector3d();
    gzwarn << "[gazebo_attitude_target_plugin] Using default camera position of " << _pose_cam_XYZ << " m\n";
  }

  if (sdf->HasElement("posCamRPY")) {
    _pose_cam_RPY = sdf->Get<ignition::math::Vector3d>("posCamRPY");
  } else {
    _pose_cam_RPY = ignition::math::Vector3d();
    gzwarn << "[gazebo_attitude_target_plugin] Using default camera rotation of " << _pose_cam_RPY << " m\n";
  }

  _pose_cam.Rot().Euler(_pose_cam_RPY);

  if (sdf->HasElement("pubRate")) {
    _pub_rate = sdf->GetElement("pubRate")->Get<int>();
  } else {
    _pub_rate = kDefaultPubRate;
    gzwarn << "[gazebo_attitude_target_plugin] Using default publication rate of " << _pub_rate << " Hz\n";
  }
}

void AttitudeTargetPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;
  _world = _model->GetWorld();
  _target = _world->ModelByName("Big box 4");

#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember target pose
  _pose_tgt = _target->WorldPose();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose
  _pose_tgt = ignitionFromGazeboMath(_target->GetWorldPose());
#endif

  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AttitudeTargetPlugin::OnUpdate, this, _1));

  _pub_att_tgt = _nh->Advertise<sensor_msgs::msgs::AttitudeTarget>("~/" + _model->GetName() + "/attitude_target", 10);
}

void AttitudeTargetPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = _world->SimTime();
  _pose_tgt = _target->WorldPose();
#else
  common::Time current_time = _world->GetSimTime();
  _pose_tgt = ignitionFromGazeboMath(_target->GetWorldPose());
#endif
  double dt = (current_time - _last_pub_time).Double();

  // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose_model = _model->WorldPose();
#else
  ignition::math::Pose3d pose_model = ignitionFromGazeboMath(_model->GetWorldPose());
#endif

  // direction to the target with respect to the model in ENU frame
  ignition::math::Pose3d pose_tgt_model;
  pose_tgt_model.Pos().X() = _pose_tgt.Pos().X() - pose_model.Pos().X();
  pose_tgt_model.Pos().Y() = _pose_tgt.Pos().Y() - pose_model.Pos().Y();
  pose_tgt_model.Pos().Z() = _pose_tgt.Pos().Z() - pose_model.Pos().Z();

  // rotate direction to the target with respect to the model from ENU to FLU
  auto pose_tgt_model_FLU = pose_model.Rot().RotateVectorReverse(pose_tgt_model.Pos());

  // add camera position into FLU to the direction to the target into FLU
  //pose_tgt_model_FLU += _pose_cam_XYZ;

  // rotate direction to the target from FLU to CAM
  //auto pose_tgt_model_CAM = _pose_cam.Rot().RotateVector(pose_tgt_model_FLU);

  auto azimuth_body = atan2(pose_tgt_model_FLU.Y(), pose_tgt_model_FLU.X());
  auto elevation_body = atan2(-pose_tgt_model_FLU.Z(), sqrt(pose_tgt_model_FLU.X()*pose_tgt_model_FLU.X() + pose_tgt_model_FLU.Z()*pose_tgt_model_FLU.Z()));
  auto tracking_status = (fabs(azimuth_body) < 0.7) && (fabs(elevation_body) < 0.7) && (pose_tgt_model_FLU.Length() < 800);

  if (dt > 1.0 / _pub_rate) {

    // Fill attitude target msg
    _att_tgt_msg.set_time_usec(current_time.Double() * 1e6);
    _att_tgt_msg.set_azimuth_rad(azimuth_body);
    _att_tgt_msg.set_elevation_rad(elevation_body);
	_att_tgt_msg.set_tracking_status(tracking_status);

    _last_pub_time = current_time;

    // publish odom msg
    _pub_att_tgt->Publish(_att_tgt_msg);
  }
}
} // namespace gazebo
