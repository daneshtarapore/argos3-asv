/**
 * @file <argos3/plugins/robots/asv/simulator/asv_entity.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef ASV_ENTITY_H
#define ASV_ENTITY_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace argos
{
   class CControllableEntity;
   class CEmbodiedEntity;
   class CASVEntity;
   class CLEDEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/propeller_equipped_entity.h>

namespace argos
{

   class CASVEntity : public CComposableEntity
   {

   public:

      ENABLE_VTABLE();

   public:

      CASVEntity();

      CASVEntity(const std::string& str_id,
                 const std::string& str_controller_id,
                 const CVector3& c_position = CVector3(),
                 const CQuaternion& c_orientation = CQuaternion());

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      inline CControllableEntity& GetControllableEntity()
      {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity()
      {
         return *m_pcEmbodiedEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity()
      {
         return *m_pcLEDEquippedEntity;
      }

      inline CPropellerEquippedEntity& GetPropellerEquippedEntity()
      {
         return *m_pcPropellerEquippedEntity;
      }

      virtual std::string GetTypeDescription() const
      {
         return "ASV";
      }

      void SetHydrodynamicParameters();

   public:

      Matrix<float, 3, 3>                    m_cBodyMassMatrix;
      Matrix<float, 3, 3>                    m_cAddedBodyMassMatrix;
      Matrix<float, 3, 1>                    m_cLinearDragCoefficients;
      Matrix<float, 3, 1>                    m_cQuadraticDragCoefficients;

   private:

      void SetLEDPosition();

   private:

      CControllableEntity*                   m_pcControllableEntity;
      CEmbodiedEntity*                       m_pcEmbodiedEntity;
      CLEDEquippedEntity*                    m_pcLEDEquippedEntity;
      CPropellerEquippedEntity*              m_pcPropellerEquippedEntity;

   };

}

#endif
