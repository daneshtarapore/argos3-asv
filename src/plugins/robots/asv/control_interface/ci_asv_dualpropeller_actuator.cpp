/**
 * @file <argos3/plugins/robots/asv/control_interface/ci_asv_dualpropeller_actuator.cpp>
 *
 * @author Danesh Tarapore <daneshtarapore@gmail.com>
 */

#include "ci_asv_dualpropeller_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos
{

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * The stack must have three values in this order:
    * 1. left propeller force  (a number)
    * 2. right propeller force (a number)
    * 3. propeller angle
    */
   int LuaSetDualPropellerThrustForceAndAngle(lua_State* pt_lua_state)
   {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 3)
      {
         return luaL_error(pt_lua_state, "LuaSetDualPropellerThrustForceAndAngle expects 3 arguments");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TNUMBER);
      luaL_checktype(pt_lua_state, 2, LUA_TNUMBER);
      luaL_checktype(pt_lua_state, 3, LUA_TNUMBER);

      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_ASVDualPropellerActuator>(pt_lua_state, "propellers")->
         SetPropellerThrust(lua_tonumber(pt_lua_state, 1),
                            lua_tonumber(pt_lua_state, 2));

      CLuaUtility::GetDeviceInstance<CCI_ASVDualPropellerActuator>(pt_lua_state, "propellers")->
         SetPropellerAngle(lua_tonumber(pt_lua_state, 3));
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_ASVDualPropellerActuator::CreateLuaState(lua_State* pt_lua_state)
   {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "propellers");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_propeller_forceangle", &LuaSetDualPropellerThrustForceAndAngle);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

}
