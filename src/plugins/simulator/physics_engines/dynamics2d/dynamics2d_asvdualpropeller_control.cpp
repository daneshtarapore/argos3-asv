/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_asvdualpropeller_control.cpp>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#include "dynamics2d_asvdualpropeller_control.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos
{

   /****************************************/
   /****************************************/

   CDynamics2DDualPropellerControl::CDynamics2DDualPropellerControl(CDynamics2DEngine& c_engine,
                                                                    Real f_max_force,
                                                                    Real f_max_torque,
                                                                    Matrix<float, 3, 3> mrb,
                                                                    Matrix<float, 3, 3> ma,
                                                                    Matrix<float, 3, 1> dl,
                                                                    Matrix<float, 3, 1> dq,
                                                                    Vector2f pos_left,
                                                                    Vector2f pos_right) :
      CDynamics2DVelocityControl(c_engine, f_max_force, f_max_torque),
      _mass(mrb(0,0)),
      _moment_z(mrb(2,2)),
      _mass_rigidbody(mrb),
      _mass_added(ma),
      _drag_linear(dl),
      _drag_quadratic(dq),
      _prop_pos_left(pos_left),
      _prop_pos_right(pos_right)
   {
        init();

   }

   /****************************************/
   /****************************************/ 

   void CDynamics2DDualPropellerControl :: init()
   {
       // assert propeller positions are aligned along y-axis and equidistant from x-coordinate CoM
       assert(_prop_pos_left(0) ==  _prop_pos_right(0));
       assert(_prop_pos_left(1) == -_prop_pos_right(1));

       // Compute _com_prop_distx and _com_prop_disty from _prop_pos_left and _prop_pos_right
       _com_prop_distx =  std::abs(_prop_pos_left(0));
       _com_prop_disty =  std::abs(_prop_pos_left(1));

       // verify mass matrix
       assert(_mass_rigidbody(0,0) == _mass);
       assert(_mass_rigidbody(1,1) == _mass);
       assert(_mass_rigidbody(2,2) == _moment_z);

       // verify added mass matrix coefficients
       assert(_mass_added(1,2) == _mass_added(2,1));

       // Compute total mass
        _mass_total = _mass_rigidbody + _mass_added;

       // Compute the inverse of the total mass matrix
        _inv_mass_total = _mass_total.inverse();

        // initialize position, velocity and acceleration vectors
        /*_acceleration               << 0.0f, 0.0f, 0.0f;
        _velocity                   << 0.0f, 0.0f, 0.0f;
        _position                   << 0.0f, 0.0f, 0.0f;
        _velocity_inertialframe     << 0.0f, 0.0f, 0.0f;*/


        /**
         * Enter the ASV body mass and hydrodynamic parameters for Chipmunk to use in modelling its dynamics
        */
        m_ptControlBody->asv_parameters.m = _mass;

        // make sure _inv_mass_total is a diagnoal matrix
        assert(_inv_mass_total(0,1)==0.0f); assert(_inv_mass_total(0,2)==0.0f);
        assert(_inv_mass_total(1,0)==0.0f); assert(_inv_mass_total(1,2)==0.0f);
        assert(_inv_mass_total(2,0)==0.0f); assert(_inv_mass_total(2,1)==0.0f);

        /* Inverse of total mass matrix */
        m_ptControlBody->asv_parameters.m_inv11 = _inv_mass_total(0,0);
        m_ptControlBody->asv_parameters.m_inv22 = _inv_mass_total(1,1);
        m_ptControlBody->asv_parameters.m_inv33 = _inv_mass_total(2,2);

        /* Added mass coefficients */
        m_ptControlBody->asv_parameters.xu1 = _mass_added(0,0);
        m_ptControlBody->asv_parameters.yv1 = _mass_added(1,1);
        m_ptControlBody->asv_parameters.nr1 = _mass_added(2,2);
        m_ptControlBody->asv_parameters.yr1 = _mass_added(1,2);

        /* Linear drag coefficients */
        m_ptControlBody->asv_parameters.xu2 = _drag_linear(0);
        m_ptControlBody->asv_parameters.yv2 = _drag_linear(1);
        m_ptControlBody->asv_parameters.nr2 = _drag_linear(2);

        /* Quadratic drag coefficients */
        m_ptControlBody->asv_parameters.xu3 = _drag_quadratic(0);
        m_ptControlBody->asv_parameters.yv3 = _drag_quadratic(1);
        m_ptControlBody->asv_parameters.nr3 = _drag_quadratic(2);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DDualPropellerControl :: set_thrust(float fl, float fr, float angle)
   {
       /*COMPUTE THE THRUST MATRIX FROM THE PROPELLER ANGLE, PROPELLER FORCES AND PROPELLER POSITION WRT CoM*/
       //_thrust << (fl + fr) * std::cos(angle), (fl + fr) * std::sin(angle), (-fl + fr) * _com_prop_disty * std::cos(angle) + (fl + fr) * _com_prop_distx * std::sin(angle);

       /* Coefficients of ASV thrust vector */
       m_ptControlBody->asv_parameters.t_11 = (fl + fr) * std::cos(angle);
       m_ptControlBody->asv_parameters.t_21 = (fl + fr) * std::sin(angle);
       m_ptControlBody->asv_parameters.t_31 = (-fl + fr) * _com_prop_disty * std::cos(angle) + (fl + fr) * _com_prop_distx * std::sin(angle);


       //m_ptControlledBody->asv_parameters = m_ptControlBody->asv_parameters;

       //printf("\n set_thrust m_ptControlBody->asv_parameters.tt: <%f %f %f>; \n",m_ptControlBody->asv_parameters.t_11, m_ptControlBody->asv_parameters.t_21, m_ptControlBody->asv_parameters.t_31);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DDualPropellerControl::SetPropellerThrustAndAngle(Real f_left_propeller_thrust, Real f_right_propeller_thrust,
                                                                    Real f_left_propeller_angle, Real f_right_propeller_angle)
   {
       assert(f_left_propeller_angle == f_right_propeller_angle);
       set_thrust(f_left_propeller_thrust, f_right_propeller_thrust, f_left_propeller_angle);


        std::cout << " thrust " << f_left_propeller_thrust << "  " << f_right_propeller_thrust << " angle " <<  f_right_propeller_angle << std::endl;


       //inverseDynamics();
       //kinematics();

       /*std::cout << " angular acceleration " << _acceleration(2) << std::endl;
       std::cout << " angular velocity " << _velocity(2) << std::endl;
       std::cout << " angular velocity inertial frame " << _velocity_inertialframe(2) << std::endl;*/

       //SetAngularVelocity(_velocity_inertialframe(2));
       //SetLinearVelocity(CVector2(_velocity_inertialframe(0), _velocity_inertialframe(1)));
   }

   /****************************************/
   /****************************************/

//   void CDynamics2DDualPropellerControl :: inverseDynamics()
//   {
//       /*std::cout << std::endl << "mass total " << _mass_total;
//       std::cout << std::endl << "mass rigid body " << _mass_rigidbody;
//       std::cout << std::endl << "mass added " << _mass_added;
//       std::cout << "mass total inverse " << _mass_total.inverse();
//       std::cout << "acceleration " << _acceleration << std::endl;
//       std::cout << "velocity " << _velocity << std::endl;

//        std::cout << std::endl<< "thrust " << _thrust << std::endl;
//        std::cout << "drag * velocity" << _drag_linear*_velocity << std::endl;*/


//       //added mass matrix coefficients
//       float Xu1 = _mass_added(0,0);
//       float Yv1 = _mass_added(1,1);
//       float Yr1 = _mass_added(1,2);

//       // Compute Coriolis force for rigid body
//       //CRB = [0 -m*r 0; m*r 0 0; 0 0 0];
//       _coriolis_rigidbody << 0.0f, -_mass*_velocity(2), 0.0f,
//                              -_mass*_velocity(2), 0.0f, 0.0f, //WAS A MISTAKE. SHOULD BE    '+_mass*_velocity(2), 0.0f, 0.0f,'
//                              0.0f, 0.0f, 0.0f;

//       // Compute Coriolis force for added mass
//       _coriolis_added << 0.0f, 0.0f, -(Yv1*_velocity(1) + Yr1*_velocity(2)),
//                          0.0f, 0.0f, Xu1*_velocity(0),
//                          (Yv1*_velocity(1)+Yr1*_velocity(2)), -Xu1*_velocity(0), 0.0f;

//       // compute drag force from quadractic drag coefficients
//       Matrix<float, 3, 3> drag_quadratic;
//       drag_quadratic << _drag_quadratic(0,0)*std::abs(_velocity(0)), 0.0f, 0.0f,
//                          0.0f, _drag_quadratic(1,1)*std::abs(_velocity(1)), 0.0f,
//                          0.0f, 0.0f, _drag_quadratic(2,2)*std::abs(_velocity(2));

//       _acceleration = _mass_total.inverse() * (_thrust - _coriolis_rigidbody*_velocity - _coriolis_added*_velocity - _drag_linear*_velocity - drag_quadratic*_velocity);

////       std::cout << " angular thrust  " << _thrust(2) << std::endl;
////       std::cout << " _drag_linear " << _drag_linear << std::endl;
////       std::cout << " _velocity " << _velocity << std::endl;

////       std::cout << " drag x velocity  " <<  (_drag_linear*_velocity) << std::endl;
//       // _acceleration = _mass_total.inverse() * (_thrust - _drag_linear*_velocity);
//   }

//   /****************************************/
//   /****************************************/

//   void CDynamics2DDualPropellerControl :: kinematics()
//   {
//       //float angle = m_ptControlBody->a;

//       _position << m_ptControlledBody->p.x, m_ptControlledBody->p.y, m_ptControlledBody->a;

//       _velocity = integral(_acceleration, _velocity);
//       Matrix<float, 3, 3> jacobian;
//       jacobian << std::cos(m_ptControlledBody->a), -std::sin(m_ptControlledBody->a), 0.0,
//                   std::sin(m_ptControlledBody->a),  std::cos(m_ptControlledBody->a), 0.0,
//                   0.0, 0.0, 1.0;

//       _velocity_inertialframe = jacobian * _velocity;
//       _position = integral(_velocity_inertialframe, _position);

//       std::cout << " m_ptControlledBody->p " << m_ptControlledBody->p.x << "    " << m_ptControlledBody->p.y <<  " m_ptControlledBody->a " << m_ptControlledBody->a << std::endl;



//       //m_ptControlledBody->p = cpv(_position(0), _position(1));
//       //cpBodySetAngle(m_ptControlledBody, _position(2));
//   }

//   /****************************************/
//   /****************************************/

//   Vector3f CDynamics2DDualPropellerControl :: integral(Vector3f & x_dot, Vector3f & x)
//   {
//       return x + _delta_t * x_dot;
//   }

}


