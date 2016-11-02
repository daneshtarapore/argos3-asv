
/**
 * @file <argos3/plugins/robots/asv/control_interface/ci_asv_dualpropeller_actuator.h>
 *
 * @author Danesh Tarapore <daneshtarapore@gmail.com>
 */
#ifndef CCI_ASV_DUALPROPELLER_ACTUATOR_H
#define CCI_ASV_DUALPROPELLER_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos
{
   class CCI_ASVDualPropellerActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>

namespace argos
{

   class CCI_ASVDualPropellerActuator : public CCI_Actuator
   {

   public:

      virtual ~CCI_ASVDualPropellerActuator() {}

      virtual void SetPropellerThrust(Real f_left_propeller_thrust,
                                      Real f_right_propeller_thrust) = 0;

      virtual void SetPropellerAngle(Real f_propeller_angle) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   protected:

      Real m_fCurrentPropellerThrust[2];
      Real m_fCurrentPropellerAngle[2];

   };

}

#endif
