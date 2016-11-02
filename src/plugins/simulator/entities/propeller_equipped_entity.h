/**
 * @file <argos3/plugins/simulator/entities/propeller_equipped_entity.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

#ifndef PROPELLER_EQUIPPED_ENTITY_H
#define PROPELLER_EQUIPPED_ENTITY_H

#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos
{

   class CPropellerEquippedEntity : public CEntity
   {

   public:

      ENABLE_VTABLE();

   public:

      CPropellerEquippedEntity(CComposableEntity* pc_parent,
                           size_t un_num_propellers);

      CPropellerEquippedEntity(CComposableEntity* pc_parent,
                           const std::string& str_id,
                           size_t un_num_propellers);

      virtual ~CPropellerEquippedEntity();

      virtual void Reset();

      inline size_t GetNumPropellers() const
      {
         return m_unNumPropellers;
      }

      void SetPropeller(UInt32 un_index,
                        const CVector3& c_position);

      const CVector3& GetPropellerPosition(size_t un_index) const;

      inline const CVector3* GetPropellerPositions() const
      {
         return m_pcPropellerPositions;
      }

      Real GetPropellerThrust(size_t un_index) const;

      inline const Real* GetPropellerThrust() const
      {
         return m_pfPropellerThrust;
      }

      void SetPropellerThrust(Real* pf_force);

      Real GetPropellerAngle(size_t un_index) const;

      inline const Real* GetPropellerAngle() const
      {
         return m_pfPropellerAngle;
      }

      void SetPropellerAngle(Real* pf_angle);



      virtual std::string GetTypeDescription() const
      {
         return "propellers";
      }

   private:

      size_t m_unNumPropellers;
      CVector3* m_pcPropellerPositions;
      Real* m_pfPropellerThrust;
      Real* m_pfPropellerAngle;

   };

}

#endif
