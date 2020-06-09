/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include <chrono>
#include <ctime>
#include <list>
#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;
  this->is_right_wing=0;



  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 0.0;
}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  //std::cout<<" angle of attack , position dans l'espace ,  rightwing, force totale, cl, cd "<<std::endl;

  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("a"))
    this->alpha = _sdf->Get<double>("a");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("is_right_wing"))
    this->is_right_wing = _sdf->Get<bool>("is_right_wing");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");


  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velWind = this->link->WorldWindLinearVel();
  ignition::math::Vector3d vel =this->link->WorldLinearVel(this->cp) ;
#else
  ignition::math::Vector3d velWind = ignitionFromGazeboMath(this->link->GetWorldWindLinearVel(this->cp));
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
#endif
  ignition::math::Vector3d velI = vel- velWind;
  //std::cout << "velwind: " << vel[0] << " " << vel[1] << " " << vel[2] << std::endl ;


  if (velI.Length() <= 0.05)
    {
      return;
    }
  velI.Normalize();
  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;
  // pose of body
  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  this->sweep = asin(sinSweepAngle);
  double cosSweepAngle2 = sqrt(cosSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*velI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of lift

  ignition::math::Vector3d liftY = spanwiseI;

  // Reverse J for the right wing
 if (this->is_right_wing)
  {
     liftY = -liftY;
  }
  liftY.Normalize();


  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
  {
      this->alpha = + acos(cosAlpha);
  }
  else
  {
      this->alpha = - acos(cosAlpha);
  }


  //modify alpha per control joint value
  //double l1 = 0.07;
  //double l2 = 0.03;
  double controlAngle=0.0;
//  ignition::math::Vector3d AF;

  if (this->controlJoint)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    controlAngle = this->controlJoint->Position(0);
#else
    controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
  }
  //double corrAngle=- atan2( l2 * sin(controlAngle), l1  + l2 * cos(controlAngle));
  //this->alpha = this->alpha + corrAngle;

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;


  // Compute sigma0
  double sigma0;
  double deltas = (this->alphaStall) /3.0;

  if (this->alpha > - this->alphaStall)
  {
      if (this->alpha < this->alphaStall )
      {
          sigma0 = 1.0;
      }
      else
      {
          if (this->alpha < alphaStall + deltas)
          {
              sigma0 = 0.5 * ( 1.0 + cos ( M_PI * (this->alpha - this->alphaStall ) / deltas ));
          }
          else
          {
              sigma0 = 0;
          }
      }

  }
  else
  {
      if (this->alpha > - this->alphaStall - deltas)
      {
          sigma0 = 0.5 * ( 1.0 + cos ( M_PI * (this->alpha + this->alphaStall ) / deltas ));
      }
      else
      {
          sigma0 = 0;
      }
   }

  // compute cd at cp, check for stall, correct for sweep
  double cd1fp = this->cdaStall;
  double cd0 = 0.03154;
  double AspectRatio =6.0;
  double cl1sa =  2 * M_PI *( AspectRatio / ( 2 + AspectRatio ));
  double cd1sa =  this->cda ; //cl1sa * cl1sa * (0.007 + ( 1.05 / ( M_PI * AspectRatio)));

  double Clsa = 0.5 * cl1sa * sin(2.0 * (this->alpha+this->alpha0));
  double Cdsa = cd0 + ( cd1sa * sin(this->alpha+this->alpha0) * sin(this->alpha+this->alpha0));
  double Clfp = 0.5 * cd1fp * sin(2.0 * (this->alpha));
  double Cdfp = cd0 + (cd1fp * sin(this->alpha) * sin(this->alpha));
  double cd = ((sigma0*(Cdsa - Cdfp)) + Cdfp );
  double cl = ((sigma0*(Clsa - Clfp)) + Clfp );

  cl = cl + this->controlJointRadToCL * controlAngle;


  cd = std::abs(cd);

  ignition::math::Vector3d lift = (cl * q * this->area * liftI * cosSweepAngle) + (Clfp * q * this->area * liftY *sinSweepAngle);
  ignition::math::Vector3d drag = cd * q * this->area * dragDirection * cosSweepAngle;

  //std::cout<<cl1sa<<" , "<<" Cl1sa "<<" , "<< cd1sa<<" , "<<" Cd1sa "<<" , "<<this->is_right_wing<<" , "<<Cdfp<<" , "<< " Cdfp "<<" , "<<Clfp<<" , "
  //        <<" Clfp "<<" , "<<cl<<" , "<<" cl et cd  "<<" , "<<cd<<" , "<<this->alpha<<" , "<<" alpha"<<std::endl;
  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * this->alpha * cosSweepAngle;

  /// \TODO: implement cm
  /// for now, reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
#else
  ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
#endif

  // moment arm from cg to cp in inertial plane
  ignition::math::Vector3d momentArm = pose.Rot().RotateVector(
    this->cp - cog
  );
  // gzerr << this->cp << " : " << this->link->GetInertial()->CoG() << "\n";


  // force and torque about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;

//  std::cout<<this->alpha<<" , "<< pose<<" , "<<this->is_right_wing<< " , "<<force<<" , "<<cl<<" , "<<cd<<std::endl;


  // + moment.Cross(momentArm);

  ignition::math::Vector3d torque = moment;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (0)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftI << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "
          << cd << " cda: " << this->cda << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "cp momentArm: " << momentArm << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "torque: " << torque << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  //this->link->AddTorque(torque);

}
