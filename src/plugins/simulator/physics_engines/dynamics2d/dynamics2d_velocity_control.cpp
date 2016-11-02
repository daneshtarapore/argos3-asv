/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_velocity_control.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_velocity_control.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics2DVelocityControl::CDynamics2DVelocityControl(CDynamics2DEngine& c_engine,
                                                  Real f_max_force,
                                                  Real f_max_torque) :
      m_cDyn2DEngine(c_engine),
      m_ptControlBody(NULL),
      m_ptControlledBody(NULL),
      m_ptLinearConstraint(NULL),
      m_ptAngularConstraint(NULL),
      m_fMaxForce(f_max_force),
      m_fMaxTorque(f_max_torque)
   {
      //m_ptControlBody = cpBodyNew(INFINITY, INFINITY);
      m_ptControlBody = cpSpaceAddControlBody(c_engine.GetPhysicsSpace(), cpBodyNew(INFINITY, INFINITY));
   }

   /****************************************/
   /****************************************/

   CDynamics2DVelocityControl::~CDynamics2DVelocityControl()
   {
      cpSpaceRemoveControlBody(m_cDyn2DEngine.GetPhysicsSpace(), m_ptControlBody);
      cpBodyFree(m_ptControlBody);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DVelocityControl::AttachTo(cpBody* pt_body)
   {
      /* If we are already controlling a body, detach from it first */
      if(m_ptControlledBody != NULL)
      {
         std::cout << "Detach and again attach to control body " << std::endl;

         Detach();
      }
      /* Set the wanted body as the new controlled one */
      m_ptControlledBody = pt_body;

      /* m_ptControlBody (the kinetic body) and m_ptControlledBody (dynamic body) need to be linked below for m_ptControlBody to have access to the orientation of the m_ptControlledBody (so as to convert velocity vector from local to world coordinate frame)*/
      m_ptControlBody->ControlledBody = m_ptControlledBody;


      /* Add linear constraint */
      m_ptLinearConstraint =
         cpSpaceAddConstraint(m_cDyn2DEngine.GetPhysicsSpace(),
                              cpPivotJointNew2(m_ptControlledBody,
                                               m_ptControlBody,
                                               cpvzero,
                                               cpvzero));
      m_ptLinearConstraint->maxBias = 0.0f; /* disable joint correction */
      m_ptLinearConstraint->maxForce = m_fMaxForce; /* limit the dragging force */
      /* Add angular constraint */
      m_ptAngularConstraint =
         cpSpaceAddConstraint(m_cDyn2DEngine.GetPhysicsSpace(),
                              cpGearJointNew(m_ptControlledBody,
                                             m_ptControlBody,
                                             0.0f,
                                             1.0f));
      m_ptAngularConstraint->maxBias = 0.0f; /* disable joint correction */
      m_ptAngularConstraint->maxForce = m_fMaxTorque; /* limit the torque */

      m_ptControlledBody->bodytype = -111;
      m_ptControlBody->bodytype = -10;

   }

   /****************************************/
   /****************************************/

   void CDynamics2DVelocityControl::Detach()
   {
      if(m_ptControlledBody != NULL) {
         /* Remove constraints */
         cpSpaceRemoveConstraint(m_cDyn2DEngine.GetPhysicsSpace(), m_ptLinearConstraint);
         cpSpaceRemoveConstraint(m_cDyn2DEngine.GetPhysicsSpace(), m_ptAngularConstraint);
         /* Free memory up */
         cpConstraintFree(m_ptLinearConstraint);
         cpConstraintFree(m_ptAngularConstraint);
         /* Erase pointer to controlled body */
         m_ptControlledBody = NULL;
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics2DVelocityControl::Reset()
   {
      m_ptControlBody->v.x = 0.0f;
      m_ptControlBody->v.y = 0.0f;
      m_ptControlBody->w = 0.0f;

      m_ptControlBody->vl.x = 0.0f;
      m_ptControlBody->vl.y = 0.0f;
      m_ptControlBody->w = 0.0f;

      m_ptControlBody->asv_parameters.t_11 = 0.0f;
      m_ptControlBody->asv_parameters.t_21 = 0.0f;
      m_ptControlBody->asv_parameters.t_31 = 0.0f;
   }

   /****************************************/
   /****************************************/

   CVector2 CDynamics2DVelocityControl::GetLinearVelocity() const
   {
      return CVector2(m_ptControlledBody->v.x,
                      m_ptControlledBody->v.y);
   }

   /****************************************/
   /****************************************/

   Real CDynamics2DVelocityControl::GetAngularVelocity() const
   {
      return m_ptControlBody->w;
   }

   /****************************************/
   /****************************************/
}
