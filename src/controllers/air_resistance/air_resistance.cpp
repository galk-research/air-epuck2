#include "air_resistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/chipmunk.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_multi_body_object_model.h>

#include <algorithm>
#include <cmath>

using namespace argos;

/* ----- Post-step: apply accumulated impulse (after collisions) ----- */
struct SWindPostData {
   cpBody* body;
   cpVect  J;
};

static void ApplyAccumPostStep(cpSpace* /*space*/, void* /*key*/, void* data) {
   SWindPostData* p = static_cast<SWindPostData*>(data);
   if(p && p->body) {
      cpBodyActivate(p->body);
      cpBodyApplyImpulse(p->body, p->J, cpvzero);  // at COM
   }
   delete p;
}

/* --------------------------------------------------------------- */
void CAirResistance::Init(TConfigurationNode& t_node)
{
   /* devices */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");
   m_pcRABSens= GetSensor  <CCI_RangeAndBearingSensor>       ("range_and_bearing");
   m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>     ("range_and_bearing");

   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseCms, m_fBaseCms);

   /* read <configuration><air_resistance .../> */
   TConfigurationNode& tRoot = CSimulator::GetInstance().GetConfigurationRoot();
   TConfigurationNode  tConf = GetNode(tRoot, "configuration");
   TConfigurationNode  tAir  = GetNode(tConf, "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(tAir, "angle_deg", deg);
   GetNodeAttribute(tAir, "magnitude", mag);

   const Real rad = deg * ARGOS_PI / 180.0;
   m_cWindCms.Set(mag * std::cos(rad), mag * std::sin(rad));
}

/* --------------------------------------------------------------- */
void CAirResistance::EnsurePhysicsHandle()
{
   if(m_bBodyReady) return;

   /* Fetch this controller's entity and its dyn2d physics model */
   CEntity& cEntity = CSimulator::GetInstance().GetSpace().GetEntity(GetId());
   auto& cComposable = dynamic_cast<CComposableEntity&>(cEntity);
   auto& cEmbodied   = cComposable.GetComponent<CEmbodiedEntity>("body");

   /* --- NEW: derive self radius from AABB (meters) --- */
   {
      const SBoundingBox& bb = cEmbodied.GetBoundingBox();
      const Real width =  bb.MaxCorner.GetX() - bb.MinCorner.GetX();
      const Real depth =  bb.MaxCorner.GetY() - bb.MinCorner.GetY();
      const Real r = 0.5 * std::max(width, depth);

//LOG << "my derived self radius is:" << r << "\n";

      /* sanity window: ignore absurd/zero values; keep previous fallback if so */
      if(std::isfinite(r) && r > 0.005 /*5mm*/ && r < 0.50 /*50cm*/)
         m_fSelfRadiusM = r;
      /* else keep the existing m_fSelfRadiusM (default 0.04f) */
   }

   /* Get the generic physics model and then downcast */
   CPhysicsModel& cPhys = cEmbodied.GetPhysicsModel("dyn2d");

   /* --- Single-body (e-puck2 style) --- */
   if(auto* pcSingle = dynamic_cast<CDynamics2DSingleBodyObjectModel*>(&cPhys)) {
      m_ptBody     = pcSingle->GetBody();
      m_bBodyReady = (m_ptBody != nullptr);
      if(!m_bBodyReady) {
         THROW_ARGOSEXCEPTION("dyn2d single-body model returned null Chipmunk body for " << GetId());
      }
      return;
   }

   /* --- Multi-body (foot-bot, etc.) --- */
   if(auto* pcMulti = dynamic_cast<CDynamics2DMultiBodyObjectModel*>(&cPhys)) {
      /* Heuristic: take body #0 (main chassis in stock robots) */
      cpBody* pMain = pcMulti->GetBody(0).Body;   // SBody::Body is cpBody*
      if(!pMain) {
         THROW_ARGOSEXCEPTION("dyn2d multi-body model body[0] is null for " << GetId());
      }
      m_ptBody     = pMain;
      m_bBodyReady = true;
      return;
   }

   /* Neither single nor multi */
   THROW_ARGOSEXCEPTION("No compatible dyn2d model (single/multi) for " << GetId());
}

/* --------------------------------------------------------------- */
Real CAirResistance::GetYawRadians() const
{
   CRadians cZ, cY, cX;
   m_pcPos->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
   return cZ.GetValue();
}

/* --------------------------------------------------------------- */
bool CAirResistance::IsBlockedByRAB(Real& fOutReduction) const
{
   /*
    * Determine whether any upwind neighbor aerodynamically shields this robot,
    * and by how much to reduce the global wind as a result.
    *
    * Output:
    *   fOutReduction ∈ [0, 1]
    *     0.0 => no shielding (full wind)
    *     1.0 => full shielding (no wind)
    *
    * Model overview (per neighbor):
    *   - Treat the upwind neighbor as casting a downwind "shadow".
    *   - Shadow strength = LateralCoverage * ShadowFade, then (optionally) gamma-boosted.
    *
    * Lateral coverage (sideways tolerance):
    *   - Gaussian in the lateral offset measured in units of blocker radii.
    *   - lateral_reach_radii ≈ how many radii give strong coverage sideways (typ. 1.5–2.5).
    *
    * Shadow fade (downwind reach):
    *   - Smoothstep over distance measured in units of blocker radii.
    *   - shadow_length_radii ≈ distance (in radii) until the effect fades to ~0 (typ. 3–5).
    *
    * Notes:
    *   - All geometry is in meters.
    *   - RAB range arrives in centimeters -> converted to meters here.
    *   - Neighbor may advertise an effective radius in Data[0] as millimeters.
    *     If missing/absurd, we fall back to m_fSelfRadiusM.
    *   - We take the maximum reduction across valid upwind neighbors.
    */

   fOutReduction = 0.0;

   /* Guards */
   if(!m_pcPos || !m_pcRABSens) return false;
   if(m_cWindCms.Length() < 1e-9) return false;

   /* --- Tunables (geometry-space; not exposed via XML) --- */
   const Real lateral_reach_radii  = 3.0;  /* lateral Gaussian reach in blocker radii */
   const Real shadow_length_radii  = 4.0;  /* downwind fade length in blocker radii */
   const Real gamma_boost          = 2.0;  /* non-linear boost: 1 - (1 - x)^gamma_boost ; 1.0 disables */

   /* Robustness gate: require a minimum upwind separation to avoid side-by-side false positives.
      A follower must be at least this many radii behind the blocker before any wake applies. */
   const Real upwind_gate_radii    = 0.5;  /* ≥0.5 radii recommended for e-puck2 */

   /* Sanity window for advertised radius (meters) */
   constexpr Real ADV_RADIUS_MIN_M = 0.005; /* 5 mm  */
   constexpr Real ADV_RADIUS_MAX_M = 0.20;  /* 20 cm */

   /* Unit wind direction in world frame */
   CVector2 wind_dir_world = m_cWindCms;
   wind_dir_world.Normalize();

   const Real yaw_world = GetYawRadians();
   const auto& readings = m_pcRABSens->GetReadings();

   bool any_blocker = false;

   for(const auto& rcv : readings) {
      /* Range (cm -> m) */
      const Real range_cm = rcv.Range;
      if(range_cm <= 1e-6) continue;
      const Real range_m = range_cm * 0.01;

      /* Blocker radius (advertised mm -> m), sanity-clamped */
      Real blocker_radius_m = m_fSelfRadiusM; /* default (~0.04 m for e-puck2) */
      if(rcv.Data.Size() >= 1) {
         const Real advertised_radius_m = static_cast<Real>(rcv.Data[0]) * 0.001;
         if(advertised_radius_m > ADV_RADIUS_MIN_M && advertised_radius_m < ADV_RADIUS_MAX_M) {
            blocker_radius_m = advertised_radius_m;
         }
      }

      /* Bearing to neighbor -> world frame */
      const Real bearing_world = yaw_world + rcv.HorizontalBearing.GetValue();

      /* Vector ME -> OTHER (m, world) */
      const CVector2 me_to_other_m(range_m * std::cos(bearing_world),
                                   range_m * std::sin(bearing_world));

      /* Vector OTHER -> ME (from neighbor toward me) */
      const CVector2 other_to_me_m = -me_to_other_m;

      /* Along-wind component (m): only consider UPWIND neighbors (positive projection) */
      const Real along_wind_m = other_to_me_m.DotProduct(wind_dir_world);

      /* --- Upwind gate: reject side-by-side or nearly-equal-X neighbors --- */
      const Real gate_m = std::max<Real>(1e-6, upwind_gate_radii * blocker_radius_m);
      if(along_wind_m <= gate_m) continue; /* not meaningfully upwind -> no shielding */

      /* Lateral offset from wind centerline (m) */
      const CVector2 lateral_vec_m   = other_to_me_m - (wind_dir_world * along_wind_m);
      const Real     lateral_offset_m = lateral_vec_m.Length();

      /* ---------- Lateral coverage: Gaussian over radii ---------- */
      const Real lateral_sigma_m = std::max<Real>(1e-6, lateral_reach_radii * blocker_radius_m);
      const Real lateral_norm    = lateral_offset_m / lateral_sigma_m;
      const Real lateral_coverage = std::exp(-0.5 * lateral_norm * lateral_norm);
      if(lateral_coverage <= 1e-6) continue; /* too far sideways */

      /* ---------- Shadow fade: smoothstep over shadow_length_radii ----------
         Start the fade AFTER the upwind gate, so being just beyond the gate gives full strength (s=0). */
      const Real fade_denominator_m = std::max<Real>(1e-6, shadow_length_radii * blocker_radius_m);
      const Real s_raw = (along_wind_m - gate_m) / fade_denominator_m;
      const Real fade_param = std::clamp(s_raw, 0.0, 1.0);
      /* smoothstep: 1 at 0 (just past the gate), 0 at 1 (beyond gate + shadow length) */
      const Real shadow_fade = 1.0 - (3.0 * fade_param * fade_param - 2.0 * fade_param * fade_param * fade_param);

      /* ---------- Combine & optional gamma remap ---------- */
      Real reduction = lateral_coverage * shadow_fade;

      if(gamma_boost != 1.0) {
         /* map x -> 1 - (1 - x)^gamma_boost ; boosts mid-range without exceeding 1 */
         const Real one_minus = std::max<Real>(0.0, 1.0 - reduction);
         reduction = 1.0 - std::pow(one_minus, std::max<Real>(1.0, gamma_boost));
      }

      reduction = std::clamp(reduction, 0.0, 1.0);

      fOutReduction = std::max(fOutReduction, reduction);
      any_blocker   = true;

      /* Optional debug:
      // LOG << "[blk] along=" << along_wind_m
      //     << " lat=" << lateral_offset_m
      //     << " r=" << blocker_radius_m
      //     << " lat_cov=" << lateral_coverage
      //     << " fade=" << shadow_fade
      //     << " red=" << reduction << std::endl;
      */
   }

   return any_blocker && fOutReduction > 1e-6;
}

/* --------------------------------------------------------------- */
CVector2 CAirResistance::ComputeEffectiveWind() const
{
   if(m_cWindCms.Length() < 1e-9) return m_cWindCms;

   Real red = 0.0;
   if(IsBlockedByRAB(red)) {
      const Real mult = std::max(0.0, 1.0 - red);
      return m_cWindCms * mult;
   }
   return m_cWindCms;
}

/* --------------------------------------------------------------- */
/* Adds wind impulse to the per-tick accumulator */
void CAirResistance::ApplyWindImpulse()
{
   const CVector2 eff = ComputeEffectiveWind();
   if(eff.Length() < 1e-9) return;

   const Real mass = cpBodyGetMass(m_ptBody);
   const CVector2 Jv = (eff / 100.0) * mass;

   m_cAccumImpulse += Jv;
}

/* Adds drive impulse (forward along yaw) to the per-tick accumulator */
void CAirResistance::DriveImpulse(Real velocity_cm_s)
{
   const Real yaw = GetYawRadians();
   const CVector2 fwd(std::cos(yaw), std::sin(yaw));

   const CVector2 v_cm_s = fwd * velocity_cm_s;
   const Real      mass  = cpBodyGetMass(m_ptBody);
   const CVector2  Jv    = (v_cm_s / 100.0) * mass;

   m_cAccumImpulse += Jv;
}

/* --------------------------------------------------------------- */
/* PRE: reset + wind + broadcast; no scheduling here */
void CAirResistance::HandleAerodynamicsPreStep()
{
	
   EnsurePhysicsHandle();

   /* reset per-tick accumulator */
   m_cAccumImpulse.Set(0.0f, 0.0f);

   /* wind contribution */
   ApplyWindImpulse();

   /* RAB broadcast: radius in mm */
   if(m_pcRABAct) 
   {
   const UInt8 r_mm = static_cast<UInt8>(
      std::min<Real>(255.0, std::round(m_fSelfRadiusM * 1000.0)));

   /* Robot-agnostic: write just byte #0; no size mismatch regardless of total RAB size */
   m_pcRABAct->SetData(0, r_mm);

   /* (Optional) If you want to ensure the rest is zero on every tick and your build exposes
      the data size accessor, you could loop i=1..N-1 and SetData(i,0). Not required though;
      by default the remaining bytes are already 0. */
   }
}

/* POST: schedule one post-step to apply the summed impulse */
void CAirResistance::HandleAerodynamicsPostStep()
{
   auto& dyn2d = dynamic_cast<CDynamics2DEngine&>(
      CSimulator::GetInstance().GetPhysicsEngine("dyn2d"));
   cpSpace* space = dyn2d.GetPhysicsSpace();

   auto* payload = new SWindPostData{
      m_ptBody,
      cpv(m_cAccumImpulse.GetX(), m_cAccumImpulse.GetY())
   };
   /* key = body → called once per body after step */
   cpSpaceAddPostStepCallback(space, ApplyAccumPostStep, m_ptBody, payload);
}

/* --------------------------------------------------------------- */
void CAirResistance::ControlStep()
{
   /* 1) before drive: reset, add wind, broadcast RAB */
   HandleAerodynamicsPreStep();

   /* 2) drive adds to accumulator */
   DriveImpulse(m_fBaseCms);

   /* 3) schedule a single post-step to apply (wind + drive) */
   HandleAerodynamicsPostStep();
}

/* --------------------------------------------------------------- */
REGISTER_CONTROLLER(CAirResistance, "air_resistance_controller")

