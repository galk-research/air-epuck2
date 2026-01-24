#include "air_resistance.h"

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/angles.h>
#include <algorithm>   // std::clamp
#include <cmath>

using namespace argos;

/*
 * Formation Template Controller (Example)
 *
 * - Translation is ALWAYS via DriveImpulse().
 * - Rotation is OPTIONAL and done as IN-PLACE turning only:
 *     wheels = (-turn_cmd, +turn_cmd)
 *
 * RAB bytes:
 *   Data[0] reserved by CAirResistance (radius in mm)
 *   Data[1] used here as "robot numeric id"
 *
 * Roles:
 *   - leader_id identifies the leader's numeric id (Data[1]).
 *   - bearing_offset_deg lets you form shapes:
 *       0   => straight line behind leader
 *      +30  => left wing in a V
 *      -30  => right wing in a V
 */
class CFormationTemplate : public CAirResistance {
public:
   void Init(TConfigurationNode& t_node) override {
      CAirResistance::Init(t_node);

      GetNodeAttributeOrDefault(t_node, "id",                 m_unId,            m_unId);
      GetNodeAttributeOrDefault(t_node, "leader_id",          m_unLeaderId,      m_unLeaderId);
      GetNodeAttributeOrDefault(t_node, "spacing_m",          m_fSpacingM,       m_fSpacingM);
      GetNodeAttributeOrDefault(t_node, "bearing_offset_deg", m_fOffsetDeg,      m_fOffsetDeg);

      // tuning
      GetNodeAttributeOrDefault(t_node, "k_v",                m_fKv,             m_fKv);
      GetNodeAttributeOrDefault(t_node, "k_turn",             m_fKTurn,          m_fKTurn);
      GetNodeAttributeOrDefault(t_node, "turn_sat",           m_fTurnSat,        m_fTurnSat);
      GetNodeAttributeOrDefault(t_node, "turn_deadband_deg",  m_fDeadbandDeg,    m_fDeadbandDeg);
      GetNodeAttributeOrDefault(t_node, "max_mul",            m_fMaxMul,         m_fMaxMul);
   }

   void ControlStep() override {
      /* 1) reset + wind + broadcast radius (byte0) */
      HandleAerodynamicsPreStep();

      /* Broadcast our numeric id on byte1 */
      if(m_pcRABAct) {
         m_pcRABAct->SetData(1, m_unId);
      }

      /* 2) Decide speed and (optional) turning */
      Real cmd_speed_cm_s = m_fBaseCms;
      Real turn_cmd = 0.0;
      bool want_turn = false;

      if(m_unId != m_unLeaderId && m_pcRABSens) {
         const auto& readings = m_pcRABSens->GetReadings();

         bool found_leader = false;
         Real leader_range_m = 0.0;
         CRadians leader_bearing;

         for(const auto& r : readings) {
            if(r.Data.Size() >= 2 && r.Data[1] == m_unLeaderId) {
               found_leader = true;
               leader_range_m = r.Range * 0.01;        // cm -> m
               leader_bearing = r.HorizontalBearing;   // relative bearing
               break;
            }
         }

         if(found_leader) {
            // Speed control (keep spacing)
            const Real err = leader_range_m - m_fSpacingM;
            cmd_speed_cm_s = m_fBaseCms + m_fKv * err;
            cmd_speed_cm_s = std::clamp(cmd_speed_cm_s, 0.0, m_fMaxMul * m_fBaseCms);

            // Bearing control (for V formation / offsets)
            const Real desired = (m_fOffsetDeg * ARGOS_PI) / 180.0;
            const Real deadband = (m_fDeadbandDeg * ARGOS_PI) / 180.0;
            const Real bearing_err = leader_bearing.GetValue() - desired;

            if(std::abs(bearing_err) > deadband) {
               want_turn = true;
               turn_cmd = m_fKTurn * bearing_err;
               turn_cmd = std::clamp(turn_cmd, -m_fTurnSat, m_fTurnSat);
            }
         }
      }

      /* 3) OPTIONAL in-place turn only (does NOT drive forward) */
      if(m_pcWheels) {
         if(want_turn) {
            m_pcWheels->SetLinearVelocity(-turn_cmd, turn_cmd);
            m_bWasTurning = true;
         } else if(m_bWasTurning) {
            // Only stop wheels if we previously commanded a turn
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            m_bWasTurning = false;
         }
      }

      /* 4) Translation via impulse */
      DriveImpulse(cmd_speed_cm_s);

      /* 5) Apply post-step */
      HandleAerodynamicsPostStep();
   }

private:
   UInt8 m_unId       = 0;
   UInt8 m_unLeaderId = 0;

   Real  m_fSpacingM  = 0.30;
   Real  m_fOffsetDeg = 0.0;

   Real  m_fKv            = 80.0;  // (cm/s)/m
   Real  m_fKTurn         = 10.0;  // turn gain
   Real  m_fTurnSat       = 10.0;  // saturate turn cmd
   Real  m_fDeadbandDeg   = 3.0;   // don't micro-oscillate
   Real  m_fMaxMul        = 2.0;   // max speed multiplier

   bool  m_bWasTurning = false;
};

REGISTER_CONTROLLER(CFormationTemplate, "formation_template_controller")

