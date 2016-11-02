/**
 * @file <argos3/plugins/robots/e-puck/simulator/qtopengl_asv.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef QTOPENGL_ASV_H
#define QTOPENGL_ASV_H

namespace argos
{
   class CQTOpenGLASV;
   class CASVEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos
{

   class CQTOpenGLASV
   {

   public:

      CQTOpenGLASV();

      virtual ~CQTOpenGLASV();

      virtual void Draw(CASVEntity& c_entity);

   protected:

      /** Sets a green plastic material */
      void SetGreenPlasticMaterial();
      /** Sets a red plastic material */
      void SetRedPlasticMaterial();
      /** Sets a circuit board material */
      void SetCircuitBoardMaterial();
      /** Sets a colored LED material */
      void SetLEDMaterial(GLfloat f_red,
                          GLfloat f_green,
                          GLfloat f_blue);

      /** Renders the body */
      void RenderBody();
      /** A single LED of the ring */
      void RenderLED();

   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** Body display list */
      GLuint m_unBodyList;

      /** LED display list */
      GLuint m_unLEDList;

      /** Number of vertices to display the round parts
          (propellers, wheels, chassis, etc.) */
      GLuint m_unVertices;

      /* Angle gap between two leds */
      GLfloat m_fLEDAngleSlice;

   };

}

#endif
