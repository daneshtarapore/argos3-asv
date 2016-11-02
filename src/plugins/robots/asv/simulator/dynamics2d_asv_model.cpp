/**
 * @file <argos3/plugins/robots/asv/simulator/dynamics2d_asv_model.cpp>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#include "dynamics2d_asv_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos
{

   /****************************************/
   /****************************************/

   /*
    * Distance in m
    */

   static const Real ASV_RADIUS              = 0.125f;
   static const Real ASV_HEIGHT              = 0.050f;

   /* Used to set the maxForce and maxTorque of the chipmunk linear constraint pivot joint and constraint pivot joint respectively.
    * The joints are constraining the dynamic body (the asv) with the kinematic body (a body with infinite mass and moment whose moment can be controlled). Since the kinematic body does not react to collisions, and the dynamic body does not allow use to set its velocity, the pivot joint constraining the two bodies needs to be created
    * Ref: http://chipmunk-physics.net/tutorials/ChipmunkTileDemo/
    */

   static const Real ASV_MAX_FORCE           = 2.0f; // for max. acceleration 2m/s^2 and body mass 1kg.
   // Max force that will be used to move the dynamic body towards the kinematic body
   static const Real ASV_MAX_TORQUE          = 0.27; // for propeller angle at 90 degrees and distance CoM to propeller along x-axis at 0.135m;
   // Max torque that will be used to move the dynamic body towards the kinematic body


   enum ASV_PROPELLERS
   {
      ASV_LEFT_PROPELLER  = 0,
      ASV_RIGHT_PROPELLER = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DASVModel::CDynamics2DASVModel(CDynamics2DEngine& c_engine,
                                            CASVEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cASVEntity(c_entity),
      m_cPropellerEquippedEntity(m_cASVEntity.GetPropellerEquippedEntity()),
      m_cDualPropeller(c_engine, ASV_MAX_FORCE, ASV_MAX_TORQUE,
                       m_cASVEntity.m_cBodyMassMatrix,
                       m_cASVEntity.m_cAddedBodyMassMatrix,
                       m_cASVEntity.m_cLinearDragCoefficients,
                       m_cASVEntity.m_cQuadraticDragCoefficients,
                       Vector2f(m_cASVEntity.GetPropellerEquippedEntity().GetPropellerPosition(ASV_LEFT_PROPELLER).GetX(),
                                m_cASVEntity.GetPropellerEquippedEntity().GetPropellerPosition(ASV_LEFT_PROPELLER).GetY()),
                       Vector2f(m_cASVEntity.GetPropellerEquippedEntity().GetPropellerPosition(ASV_RIGHT_PROPELLER).GetX(),
                                m_cASVEntity.GetPropellerEquippedEntity().GetPropellerPosition(ASV_RIGHT_PROPELLER).GetY())),
      m_fCurrentPropellerThrust(m_cPropellerEquippedEntity.GetPropellerThrust()),
      m_fCurrentPropellerAngle(m_cPropellerEquippedEntity.GetPropellerAngle())
   {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(m_cASVEntity.m_cBodyMassMatrix(0,0),
                                  m_cASVEntity.m_cBodyMassMatrix(2,2)));

      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());

      /* Create the body shape */
      cpShape* ptShape = cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                                         cpCircleShapeNew(ptBody,
                                                          ASV_RADIUS,
                                                          cpvzero));
      ptShape->e = 0.0; // Coefficient of restitution. (elasticity). Set to 0 -- no elasticity
      ptShape->u = 0.7; // Coefficient of friction between colliding objects. Set to 0.7 -- lots of friction

      /* Constrain the actual base body to follow the dual propeller control */
      m_cDualPropeller.AttachTo(ptBody);

      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, ASV_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DASVModel::~CDynamics2DASVModel()
   {
      m_cDualPropeller.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DASVModel::Reset()
   {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDualPropeller.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DASVModel::UpdateFromEntityStatus()
   {

      /* Do we want to move? */
      /* Since the body has substantial inertia, it may be better to continually update the body's velocity*/

      m_cDualPropeller.SetPropellerThrustAndAngle(m_fCurrentPropellerThrust[ASV_LEFT_PROPELLER],
                                                  m_fCurrentPropellerThrust[ASV_RIGHT_PROPELLER],
                                                  m_fCurrentPropellerAngle[ASV_LEFT_PROPELLER],
                                                  m_fCurrentPropellerAngle[ASV_RIGHT_PROPELLER]);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CASVEntity, CDynamics2DASVModel);

   /****************************************/
   /****************************************/

}
