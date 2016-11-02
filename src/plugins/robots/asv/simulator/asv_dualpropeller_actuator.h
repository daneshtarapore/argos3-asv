/**
 * @file <argos3/plugins/simulator/actuators/asv_dualpropeller_actuator.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef ASV_DUAL_PROPELLER_ACTUATOR_H
#define ASV_DUAL_PROPELLER_ACTUATOR_H

#include <string>
#include <map>

namespace argos
{
   class CASVDualPropellerActuator;
}

#include <argos3/plugins/robots/asv/control_interface/ci_asv_dualpropeller_actuator.h>
#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/simulator/entities/propeller_equipped_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/rng.h>

namespace argos
{

   class CASVDualPropellerActuator : public CSimulatedActuator,
                                     public CCI_ASVDualPropellerActuator
   {

   public:

      enum DUALPROPELLER
      {
         LEFT_PROPELLER  = 0,
         RIGHT_PROPELLER = 1
      };

   public:

      /**
       * @brief Constructor.
       */
      CASVDualPropellerActuator();

      /**
       * @brief Destructor.
       */
      virtual ~CASVDualPropellerActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      /**
       * @brief Sets the linear force (thrust) of the two propellers.
       * Forces are expressed in N.
       *
       * @param f_left_propeller_thrust desired left propeller thrust force.
       * @param f_right_propeller_thrust desired right propeller thrust force.
       */
      virtual void SetPropellerThrust(Real f_left_propeller_thrust,
                                      Real f_right_propeller_thrust);

      virtual void SetPropellerAngle(Real f_propeller_angle);


      virtual void Update();

      virtual void Reset();

   protected:

      /** Adds noise to the desired thrust forces */
      virtual void AddGaussianNoise();

   protected:

      CPropellerEquippedEntity* m_pcPropellerEquippedEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Noise parameters, at the moment noise is Gaussian */
      Real m_fNoiseStdDeviation;

   };

}

#endif
