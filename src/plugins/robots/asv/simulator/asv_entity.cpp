/**
 * @file <argos3/plugins/robots/asv/simulator/asv_entity.cpp>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#include "asv_entity.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>


namespace argos
{

   /****************************************/
   /****************************************/

   /* All measures are in SI units*/

   /*
    * We assume for now that the ASV has a cylinder shape with the following dimensions
    *
    * Based on how this projecy proceeds the next step may be to model the 2D ASV based on Anders's ASV platform -- triange or semi-circle attachled to a rectangle using CDynamics2DMultiBodyObjectModel
    */
   static const Real BODY_RADIUS                = 0.125f;
   static const Real BODY_HEIGHT                = 0.050f;
   static const Real BODY_MASS                  = 1.0f;

   static const CRadians LED_RING_START_ANGLE   = CRadians((ARGOS_PI / 8.0f) * 0.5f);
   static const Real LED_RING_RADIUS            = BODY_RADIUS + 0.007;
   static const Real LED_RING_ELEVATION         = BODY_HEIGHT;


   /* Added mass coefficients for the ASV*/
   static const Real xu1 = 0.5f;
   static const Real yv1 = 0.5f;
   static const Real nr1 = 0.0625f;
   static const Real yr1 = 0.0f;

   /* Linear drag coefficients for the ASV */
   static const Real xu2 = 10.0f;
   static const Real yv2 = 10.0f;
   static const Real nr2 = 10.0f;

   /* Quadratic drag coefficients for the ASV-- can be ignored if ASV speed is no more than around 2m/s*/
   static const Real xu3 = 0.0f;
   static const Real yv3 = 0.0f;
   static const Real nr3 = 0.0f;

   /* The two propellers are on one end of the ASV with the propeller shaft having length of 1 cm */
   static const Real CoM_PROPELLER_DISTANCE_X   = BODY_RADIUS + 0.010f;
   /* The two propellers are on either side of the y-coordinate of the ASV's CoM, at positions CoM_PROPELLER_DISTANCE_Y and -CoM_PROPELLER_DISTANCE_Y*/
   static const Real CoM_PROPELLER_DISTANCE_Y   = 0.025f;

   /* Additional robot measurements on the robots dimensions will be added when we implement sensors on the robot */


   /****************************************/
   /****************************************/

   CASVEntity::CASVEntity() :
      CComposableEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_pcPropellerEquippedEntity(NULL)
   {
       /* Set the hydrodynamic parameters of the ASV */
       SetHydrodynamicParameters();
   }

   /****************************************/
   /****************************************/

   CASVEntity::CASVEntity(const std::string& str_id,
                          const std::string& str_controller_id,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation) :
      CComposableEntity(NULL, str_id),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_pcPropellerEquippedEntity(NULL)
   {
      try
      {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);

         /* Dual propeller entity and propeller positions (left, right) */
         m_pcPropellerEquippedEntity = new CPropellerEquippedEntity(this, "propellers_0", 2);
         AddComponent(*m_pcPropellerEquippedEntity);
         m_pcPropellerEquippedEntity->SetPropeller(0, CVector3(CoM_PROPELLER_DISTANCE_X,  CoM_PROPELLER_DISTANCE_Y, 0.0f));
         m_pcPropellerEquippedEntity->SetPropeller(1, CVector3(CoM_PROPELLER_DISTANCE_X, -CoM_PROPELLER_DISTANCE_Y, 0.0f));

         /* LED equipped entity */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
                                           LED_RING_RADIUS,
                                           LED_RING_START_ANGLE,
                                           8,
                                           m_pcEmbodiedEntity->GetOriginAnchor());

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);

         /* Update components */
         UpdateComponents();

         /* Set the hydrodynamic parameters of the ASV */
         SetHydrodynamicParameters();
      }
      catch(CARGoSException& ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CASVEntity::SetHydrodynamicParameters()
   {
        m_cBodyMassMatrix << BODY_MASS, 0.0f, 0.0f,
                             0.0f, BODY_MASS, 0.0f,
                             0.0f, 0.0f, BODY_MASS*(0.5f*(BODY_RADIUS*BODY_RADIUS));

        m_cAddedBodyMassMatrix << xu1, 0.0f, 0.0f,
                                  0.0f, yv1, yr1,
                                  0.0f, yr1, nr1;

        m_cLinearDragCoefficients << xu2, yv2, nr2;

        m_cQuadraticDragCoefficients << xu3, yv3, nr3;
   }

   /****************************************/
   /****************************************/

   void CASVEntity::Init(TConfigurationNode& t_tree)
   {
      try
      {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Dual propeller entity and propeller positions (left, right) */
         m_pcPropellerEquippedEntity = new CPropellerEquippedEntity(this, "propellers_0", 2);
         AddComponent(*m_pcPropellerEquippedEntity);
         m_pcPropellerEquippedEntity->SetPropeller(0, CVector3(CoM_PROPELLER_DISTANCE_X,  CoM_PROPELLER_DISTANCE_Y, 0.0f));
         m_pcPropellerEquippedEntity->SetPropeller(1, CVector3(CoM_PROPELLER_DISTANCE_X, -CoM_PROPELLER_DISTANCE_Y, 0.0f));

         /* LED equipped entity */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
                                           LED_RING_RADIUS,
                                           LED_RING_START_ANGLE,
                                           8,
                                           m_pcEmbodiedEntity->GetOriginAnchor());

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));

         /* Update components */
         UpdateComponents();

         /* Set the hydrodynamic parameters of the ASV */
         SetHydrodynamicParameters();
      }
      catch(CARGoSException& ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CASVEntity::Reset()
   {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CASVEntity::Destroy()
   {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

   void CASVEntity::UpdateComponents()
   {
      //UPDATE(m_pcRABEquippedEntity);
      UPDATE(m_pcLEDEquippedEntity);
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CASVEntity,
                   "asv",
                   "Danesh Tarapore [daneshtarapore@gmail.com]",
                   "1.0",
                   "The ASV robot.",
                   "The ASV is a an example of an aquatic surface vehicle robot. In its\n"
                   "current form, it has two propellers with controllable thrust angle,\n"
                   "compass, frontal touch (bump) sensors, GPS, LED ring and RAB sensors \n"
                   "More information is available at TODO: ADD URL \n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <ASV id=\"eb0\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,90,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </ASV>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the ASV to point of the\n"
                   "ASV in the arena. When the robot is untranslated and unrotated, the\n"
                   "CoM of the ASV is positioned at the origin\n"
                   "The attribute values are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the ASV. All\n"
                   "rotations are performed with respect to the CoM point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "ASV. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CASVEntity);

   /****************************************/
   /****************************************/

}
