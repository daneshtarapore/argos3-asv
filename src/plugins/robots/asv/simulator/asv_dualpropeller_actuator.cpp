/**
 * @file <argos3/plugins/simulator/actuators/asv_dualpropeller_actuator.cpp>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#include "asv_dualpropeller_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/plugins/factory.h>

namespace argos
{

   /****************************************/
   /****************************************/

   CASVDualPropellerActuator::CASVDualPropellerActuator() :
      m_pcPropellerEquippedEntity(NULL),
      m_pcRNG(NULL),
      m_fNoiseStdDeviation(0.0f)
   {
      m_fCurrentPropellerThrust[LEFT_PROPELLER]  = 0.0f;
      m_fCurrentPropellerThrust[RIGHT_PROPELLER] = 0.0f;
      m_fCurrentPropellerAngle[LEFT_PROPELLER]  = 0.0f;
      m_fCurrentPropellerAngle[RIGHT_PROPELLER]  = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::SetRobot(CComposableEntity& c_entity)
   {
      try
       {
         m_pcPropellerEquippedEntity = &(c_entity.GetComponent<CPropellerEquippedEntity>("propellers"));
         if(m_pcPropellerEquippedEntity->GetNumPropellers() != 2)
         {
            THROW_ARGOSEXCEPTION("The dual propeller actuator can be associated only to a ASV with 2 propellers");
         }
         m_pcPropellerEquippedEntity->Enable();
      }
      catch(CARGoSException& ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Error setting dual propeller actuator to entity \"" << c_entity.GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::Init(TConfigurationNode& t_tree)
   {
      try
      {
         CCI_ASVDualPropellerActuator::Init(t_tree);
         GetNodeAttributeOrDefault<Real>(t_tree, "noise_std_dev", m_fNoiseStdDeviation, 0.0f);
         if(m_fNoiseStdDeviation > 0.0f)
         {
            m_pcRNG = CRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in ASV dual propeller actuator.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::SetPropellerThrust(Real f_left_propeller_thrust,
                                                      Real f_right_propeller_thrust)
   {
      /* Thrust force in Newton (N) */
      m_fCurrentPropellerThrust[LEFT_PROPELLER]  = f_left_propeller_thrust;
      m_fCurrentPropellerThrust[RIGHT_PROPELLER] = f_right_propeller_thrust;
      /* Apply noise */
      if(m_fNoiseStdDeviation > 0.0f)
      {
         AddGaussianNoise();
      }
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::SetPropellerAngle(Real f_propeller_angle)
   {
      /* Propeller angle in radians */
      m_fCurrentPropellerAngle[LEFT_PROPELLER]  = f_propeller_angle;
      m_fCurrentPropellerAngle[RIGHT_PROPELLER] = f_propeller_angle;
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::Update()
   {
      m_pcPropellerEquippedEntity->SetPropellerThrust(m_fCurrentPropellerThrust);
      m_pcPropellerEquippedEntity->SetPropellerAngle(m_fCurrentPropellerAngle);
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::Reset()
   {
      m_fCurrentPropellerThrust[LEFT_PROPELLER]  = 0.0f;
      m_fCurrentPropellerThrust[RIGHT_PROPELLER] = 0.0f;

      m_fCurrentPropellerAngle[LEFT_PROPELLER]  = 0.0f;
      m_fCurrentPropellerAngle[RIGHT_PROPELLER] = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CASVDualPropellerActuator::AddGaussianNoise()
   {
      m_fCurrentPropellerThrust[LEFT_PROPELLER]  += m_fCurrentPropellerThrust[LEFT_PROPELLER]  * m_pcRNG->Gaussian(m_fNoiseStdDeviation);
      m_fCurrentPropellerThrust[RIGHT_PROPELLER] += m_fCurrentPropellerThrust[RIGHT_PROPELLER] * m_pcRNG->Gaussian(m_fNoiseStdDeviation);
   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CASVDualPropellerActuator,
                  "dual_propeller", "default",
                  "Danesh Tarapore [daneshtarapore@gmail.com]",
                  "1.0",
                  "The dual propeller actuator for the ASV.",
                  "This actuator controls the two propellers of the ASV. For a\n"
                  "complete description of its usage, refer to the\n"
                  "ci_asv_dualpropeller_actuator.h file.\n\n"
                  "REQUIRED XML CONFIGURATION\n\n"
                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <dual_propeller implementation=\"default\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"
                  "OPTIONAL XML CONFIGURATION\n\n"
                  "It is possible to specify noisy thrust forces in order to match the characteristics\n"
                  "of the real ASV. This can be done with the attribute: 'noise_std_dev',\n"
                  "which indicates the standard deviation of a gaussian noise applied to the\n"
                  "desired force of the propellers (not applied on the angle of the propellers).:\n\n"
                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <dual_propeller implementation=\"default\"\n"
                  "                               noise_std_dev=\"1\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n",
                  "Usable"
   );





