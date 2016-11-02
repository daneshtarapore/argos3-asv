/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_asvdualpropeller_control.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef DYNAMICS2D_ASVDUALPROPELLER_CONTROL_H
#define DYNAMICS2D_ASVDUALPROPELLER_CONTROL_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;


namespace argos
{
   class CDynamics2DEngine;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_velocity_control.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos
{

   class CDynamics2DDualPropellerControl : public CDynamics2DVelocityControl
   {

   public:

      CDynamics2DDualPropellerControl(CDynamics2DEngine& c_engine,
                                      Real f_max_force,
                                      Real f_max_torque,
                                      Matrix<float, 3, 3> mrb,
                                      Matrix<float, 3, 3> ma,
                                      Matrix<float, 3, 1> dl,
                                      Matrix<float, 3, 1> dq,
                                      Vector2f pos_left,
                                      Vector2f pos_right);

      virtual ~CDynamics2DDualPropellerControl() {}

      void SetPropellerThrustAndAngle(Real f_left_propeller_thrust, Real f_right_propeller_thrust,
                                      Real f_left_propeller_angle, Real f_right_propeller_angle);

      void init();

      //void inverseDynamics();

      //Vector3f integral(Vector3f & x_dot, Vector3f & x);

      //void kinematics();

      void set_thrust(float fl, float fr, float angle);

   private:
      float _mass, _moment_z;
      Matrix<float, 3, 3> _mass_rigidbody, _mass_added, _mass_total, _inv_mass_total;
      /*Matrix<float, 3, 3> _coriolis_rigidbody, _coriolis_added;*/
      Matrix<float, 3, 1> _drag_linear, _drag_quadratic;
      //Vector3f _thrust; //float _angle_prop;
      //Vector3f _acceleration, _velocity, _position; // acceleration and velocity are in the body's frame of reference. positions is int the inertial frame of reference
      //Vector3f _velocity_inertialframe;

      Vector2f _prop_pos_left, _prop_pos_right; float _com_prop_distx, _com_prop_disty;

      //float _delta_t;
   };

}

#endif
