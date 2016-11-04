#include "asv_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/asv/simulator/asv_entity.h>
#include <controllers/asv_test/asv_test.h>

/****************************************/
/****************************************/

CASVLoopFunctions::CASVLoopFunctions() :
   m_pcFloor(NULL)
{
}

/****************************************/
/****************************************/

void CASVLoopFunctions::Init(TConfigurationNode& t_node)
{
}

/****************************************/
/****************************************/

void CASVLoopFunctions::Reset()
{

}

/****************************************/
/****************************************/

void CASVLoopFunctions::Destroy()
{

}

/****************************************/
/****************************************/

CColor CASVLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane)
{
   return CColor::BLUE;
}

/****************************************/
/****************************************/

void CASVLoopFunctions::PreStep()
{
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CASVLoopFunctions, "asv_loop_functions")
