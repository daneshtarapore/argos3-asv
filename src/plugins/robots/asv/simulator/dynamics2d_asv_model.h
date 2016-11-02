/**
 * @file <argos3/plugins/robots/asv/simulator/dynamics2d_asv_model.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef DYNAMICS2D_ASV_MODEL_H
#define DYNAMICS2D_ASV_MODEL_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;


namespace argos
{
   class CDynamics2DDualPropellerControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DASVModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_asvdualpropeller_control.h>
#include <argos3/plugins/robots/asv/simulator/asv_entity.h>

namespace argos
{

   class CDynamics2DASVModel : public CDynamics2DSingleBodyObjectModel
   {

   public:

      CDynamics2DASVModel(CDynamics2DEngine& c_engine,
                          CASVEntity& c_entity);
      virtual ~CDynamics2DASVModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      CASVEntity& m_cASVEntity;

      CPropellerEquippedEntity& m_cPropellerEquippedEntity;

      CDynamics2DDualPropellerControl m_cDualPropeller;

      const Real* m_fCurrentPropellerThrust;

      const Real* m_fCurrentPropellerAngle;
   };

}

#endif
