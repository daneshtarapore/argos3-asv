#ifndef ASV_QT_USER_FUNCTIONS_H
#define ASV_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/asv/simulator/asv_entity.h>

using namespace argos;

class CASVQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CASVQTUserFunctions();

   virtual ~CASVQTUserFunctions() {}

   void Draw(CASVEntity& c_entity);
   
};

#endif
