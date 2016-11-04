#include "asv_qt_user_functions.h"
#include <controllers/asv_test/asv_test.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CASVQTUserFunctions::CASVQTUserFunctions()
{
   RegisterUserFunction<CASVQTUserFunctions,CASVEntity>(&CASVQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CASVQTUserFunctions::Draw(CASVEntity& c_entity)
{
   CASVTest& cController = dynamic_cast<CASVTest&>(c_entity.GetControllableEntity().GetController());
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CASVQTUserFunctions, "asv_qt_user_functions")
