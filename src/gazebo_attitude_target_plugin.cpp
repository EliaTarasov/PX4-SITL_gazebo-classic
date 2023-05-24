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
    gzerr << "[gazebo_vision_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    _pub_rate = sdf->GetElement("pubRate")->Get<int>();
  } else {
    _pub_rate = kDefaultPubRate;
    gzwarn << "[gazebo_vision_plugin] Using default publication rate of " << _pub_rate << " Hz\n";
  }
}

void AttitudeTargetPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;
  _world = _model->GetWorld();
  _model_tgt = _world->ModelByName("Big box 4");

#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember target pose
  _pose_tgt = _model_tgt->WorldPose();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose
  _pose_tgt = ignitionFromGazeboMath(_model_tgt->GetWorldPose());
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
  _pose_tgt = _model_tgt->WorldPose();
#else
  common::Time current_time = _world->GetSimTime();
  _pose_tgt = ignitionFromGazeboMath(_model_tgt->GetWorldPose());
#endif
  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = _model->WorldPose();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(_model->GetWorldPose());
#endif
    ignition::math::Pose3d pose_tgt_model; // pose of a target with respect to the model in ENU frame
    pose_tgt_model.Pos().X() = _pose_tgt.Pos().X() - pose_model_world.Pos().X();
    pose_tgt_model.Pos().Y() = _pose_tgt.Pos().Y() - pose_model_world.Pos().Y();
    pose_tgt_model.Pos().Z() = _pose_tgt.Pos().Z() - pose_model_world.Pos().Z();

    // pose of a target with respect to the model in FLU frame
    ignition::math::Vector3d pose_tgt_model_flu = pose_model_world.Rot().Inverse().RotateVector(pose_tgt_model.Pos());

    //auto fl_norm = sqrt(pow(pose_tgt_model_flu.X(), 2) + pow(pose_tgt_model_flu.Y(), 2));
    auto azimuth = atan2(pose_tgt_model_flu.Y(), pose_tgt_model_flu.X());

    //auto fu_norm = sqrt(pow(pose_tgt_model_flu.X(), 2) + pow(pose_tgt_model_flu.Z(), 2));
    auto elevation = atan2(pose_tgt_model_flu.Z(), pose_tgt_model_flu.X());

    /*
    pose_model.Rot().Euler(pose_model_world.Rot().Roll(),
                           pose_model_world.Rot().Pitch(),
                           pose_model_world.Rot().Yaw());

    std::cout << "target F pose in vehicle frame " << pose_tgt_model_flu.X() << std::endl;
    std::cout << "target L pose in vehicle frame " << pose_tgt_model_flu.Y() << std::endl;
    std::cout << "target U pose in vehicle frame " << pose_tgt_model_flu.Z() << std::endl;
    */

    /*
    std::cout << "target F pose in vehicle frame " << pose_tgt_model_flu.X() << std::endl;
    std::cout << "target L pose in vehicle frame " << pose_tgt_model_flu.Y() << std::endl;
    std::cout << std::endl;
    std::cout << "target azimuth " << azimuth << std::endl;
    std::cout << "target elevation " << elevation << std::endl;
    std::cout << std::endl;
    */

    // Fill attitude target msg
    _att_tgt_msg.set_time_usec(current_time.Double() * 1e6);
    _att_tgt_msg.set_azimuth_rad(azimuth);
    _att_tgt_msg.set_elevation_rad(elevation);

    _last_pub_time = current_time;

    // publish odom msg
    _pub_att_tgt->Publish(_att_tgt_msg);
  }
}
} // namespace gazebo
