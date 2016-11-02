/**
 * @file <argos3/plugins/simulator/entities/propeller_equipped_entity.cpp>
 *
 * v
 */

#include "propeller_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>

namespace argos
{

   /****************************************/
   /****************************************/

   CPropellerEquippedEntity::CPropellerEquippedEntity(CComposableEntity* pc_parent,
                                              size_t un_num_propellers) :
      CEntity(pc_parent),
      m_unNumPropellers(un_num_propellers)
   {
      m_pcPropellerPositions = new CVector3[m_unNumPropellers];
      ::memset(m_pcPropellerPositions, 0, m_unNumPropellers * sizeof(CVector3));

      m_pfPropellerThrust = new Real[m_unNumPropellers];
      ::memset(m_pfPropellerThrust, 0, m_unNumPropellers * sizeof(Real));

      m_pfPropellerAngle = new Real[m_unNumPropellers];
      ::memset(m_pfPropellerAngle, 0, m_unNumPropellers * sizeof(Real));

      Disable();
   }

   /****************************************/
   /****************************************/

   CPropellerEquippedEntity::CPropellerEquippedEntity(CComposableEntity* pc_parent,
                                              const std::string& str_id,
                                              size_t un_num_propellers) :
      CEntity(pc_parent, str_id),
      m_unNumPropellers(un_num_propellers)
   {
      m_pcPropellerPositions = new CVector3[m_unNumPropellers];
      ::memset(m_pcPropellerPositions, 0, m_unNumPropellers * sizeof(CVector3));

      m_pfPropellerThrust = new Real[m_unNumPropellers];
      ::memset(m_pfPropellerThrust, 0, m_unNumPropellers * sizeof(Real));

      m_pfPropellerAngle = new Real[m_unNumPropellers];
      ::memset(m_pfPropellerAngle, 0, m_unNumPropellers * sizeof(Real));

      Disable();
   }

   /****************************************/
   /****************************************/

   CPropellerEquippedEntity::~CPropellerEquippedEntity()
   {
      delete[] m_pcPropellerPositions;
      delete[] m_pfPropellerThrust;
      delete[] m_pfPropellerAngle;
   }

   /****************************************/
   /****************************************/

   void CPropellerEquippedEntity::Reset()
   {
      ::memset(m_pfPropellerThrust, 0, m_unNumPropellers * sizeof(Real));
      ::memset(m_pfPropellerAngle, 0, m_unNumPropellers * sizeof(Real));
   }

   /****************************************/
   /****************************************/

   void CPropellerEquippedEntity::SetPropeller(UInt32 un_index,
                                           const CVector3& c_position)
   {
      if(un_index < m_unNumPropellers)
      {
         m_pcPropellerPositions[un_index] = c_position;
      }
      else
      {
         THROW_ARGOSEXCEPTION("CPropellerEquippedEntity::SetPropeller() : index " << un_index << " out of bounds (allowed [0:" << m_unNumPropellers << "])");
      }
   }

   /****************************************/
   /****************************************/

   const CVector3& CPropellerEquippedEntity::GetPropellerPosition(size_t un_index) const
   {
      if(un_index < m_unNumPropellers)
      {
         return m_pcPropellerPositions[un_index];
      }
      else
      {
         THROW_ARGOSEXCEPTION("CPropellerEquippedEntity::GetPropellerPosition() : index " << un_index << " out of bounds (allowed [0:" << m_unNumPropellers << "])");
      }
   }

   /****************************************/
   /****************************************/

   Real CPropellerEquippedEntity::GetPropellerThrust(size_t un_index) const
   {
      if(un_index < m_unNumPropellers)
      {
         return m_pfPropellerThrust[un_index];
      }
      else
      {
         THROW_ARGOSEXCEPTION("CPropellerEquippedEntity::GetPropellerThrust() : index " << un_index << " out of bounds (allowed [0:" << m_unNumPropellers << "])");
      }
   }

   /****************************************/
   /****************************************/

   void CPropellerEquippedEntity::SetPropellerThrust(Real* pf_force)
   {
      ::memcpy(m_pfPropellerThrust, pf_force, m_unNumPropellers * sizeof(Real));
   }

   /****************************************/
   /****************************************/

   Real CPropellerEquippedEntity::GetPropellerAngle(size_t un_index) const
   {
      if(un_index < m_unNumPropellers)
      {
         return m_pfPropellerAngle[un_index];
      }
      else
      {
         THROW_ARGOSEXCEPTION("CPropellerEquippedEntity::GetPropellerAngle() : index " << un_index << " out of bounds (allowed [0:" << m_unNumPropellers << "])");
      }
   }

   /****************************************/
   /****************************************/

   void CPropellerEquippedEntity::SetPropellerAngle(Real* pf_angle)
   {
      ::memcpy(m_pfPropellerAngle, pf_angle, m_unNumPropellers * sizeof(Real));
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CPropellerEquippedEntity);

   /****************************************/
   /****************************************/

}
