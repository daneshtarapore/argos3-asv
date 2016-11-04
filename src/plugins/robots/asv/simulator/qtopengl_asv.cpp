/**
 * @file <argos3/plugins/robots/e-puck/simulator/qtopengl_asv.cpp>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#include "qtopengl_asv.h"
#include "asv_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos
{

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real BODY_RADIUS                 = 0.125f;
   static const Real BODY_ELEVATION              = 0.000f;
   static const Real BODY_HEIGHT                 = 0.050f;                              // to be checked!

   static const Real LED_ELEVATION               = BODY_ELEVATION + BODY_HEIGHT;
   static const Real LED_HEIGHT                  = 0.01;                               // to be checked!
   static const Real LED_UPPER_RING_INNER_RADIUS = 0.8 * BODY_RADIUS;


   /****************************************/
   /****************************************/

   CQTOpenGLASV::CQTOpenGLASV() :
      m_unVertices(40),
      m_fLEDAngleSlice(360.0f / 8.0f)
   {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(2);

      /* Assign indices for better referencing (later) */
      m_unBodyList        = m_unLists;
      m_unLEDList         = m_unLists + 1;

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the LED display list */
      glNewList(m_unLEDList, GL_COMPILE);
      RenderLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLASV::~CQTOpenGLASV()
   {
      glDeleteLists(m_unLists, 2);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::Draw(CASVEntity& c_entity)
   {
      /* Place the body */
      glCallList(m_unBodyList);

      /* Place the LEDs */
      glPushMatrix();
      CLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      for(UInt32 i = 0; i < 8; i++)
      {
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         glRotatef(-m_fLEDAngleSlice, 0.0f, 0.0f, 1.0f);
         SetLEDMaterial(cColor.GetRed(),
                        cColor.GetGreen(),
                        cColor.GetBlue());
         glCallList(m_unLEDList);
      }
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::SetGrayPlasticMaterial()
   {
      const GLfloat pfColor[]     = {   0.4f, 0.4f, 0.4f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::SetGreenPlasticMaterial()
   {
      const GLfloat pfColor[]     = {   0.0f, 1.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::SetRedPlasticMaterial()
   {
      const GLfloat pfColor[]     = {   1.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::SetCircuitBoardMaterial()
   {
      const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
      const GLfloat pfShininess[] = { 10.0f                  };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::SetLEDMaterial(GLfloat f_red,
                                      GLfloat f_green,
                                      GLfloat f_blue)
   {
      const GLfloat fEmissionFactor = 10.0f;
      const GLfloat pfColor[]     = {                    f_red,                   f_green,                   f_blue, 1.0f };
      const GLfloat pfSpecular[]  = {                     0.0f,                      0.0f,                     0.0f, 1.0f };
      const GLfloat pfShininess[] = {                     0.0f                                                            };
      const GLfloat pfEmission[]  = {  f_red * fEmissionFactor, f_green * fEmissionFactor, f_blue * fEmissionFactor, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::RenderBody()
   {
      /* Set material */
      SetGrayPlasticMaterial(); //      SetGreenPlasticMaterial();

      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(-CRadians::TWO_PI / m_unVertices);

      /* Bottom part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
      }
      glEnd();

      /* Side surface */
      cAngle = -cAngle;
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();

      /* Top part */
      glBegin(GL_POLYGON);
      cVertex.Set(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();

      /* Triangle to set the direction */
      SetLEDMaterial(1.0f, 1.0f, 0.0f);
      glBegin(GL_TRIANGLES);
      glVertex3f( BODY_RADIUS * 0.7,               0.0f, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7,  BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7, -BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLASV::RenderLED()
   {
      /* Side surface */
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector2 cNormal(1.0f, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices / 8; i++)
      {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();

      /* Top surface  */
      cVertex.Set(BODY_RADIUS, 0.0f);
      CVector2 cVertex2(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices / 8; i++)
      {
         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         cVertex.Rotate(cAngle);
         cVertex2.Rotate(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawASVNormal : public CQTOpenGLOperationDrawNormal
   {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CASVEntity& c_entity)
      {
         static CQTOpenGLASV m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawASVSelected : public CQTOpenGLOperationDrawSelected
   {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CASVEntity& c_entity)
      {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawASVNormal, CASVEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawASVSelected, CASVEntity);

   /****************************************/
   /****************************************/

}
