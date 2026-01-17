#include "air_resistance.h"
#include <argos3/core/utility/math/angles.h>   // for CRadians
#include <cmath>

using namespace argos;

/*
 * A derived controller that overrides ControlStep to steer (via wheels)
 * toward a "crab" heading that compensates for crosswind, so that the
 * desired ground track is straight WEST (−X) in world coordinates.
 *
 * Translation still uses the base impulse pipeline (HandleAerodynamics* + DriveImpulse).
 */
class CWindAwareAirResistance : public CAirResistance {
public:
   virtual ~CWindAwareAirResistance() = default;

   /* Override just ControlStep; everything else comes from CAirResistance */
   void ControlStep() override {
      /* 1) run the usual pre-step: ensures physics handle, resets accumulator,
            applies wind impulse, and broadcasts RAB byte */
      HandleAerodynamicsPreStep();

      /* 2) Compute a crab heading so that:  fwd * Vd + wind_eff  ∥  track_dir
             - Use base ComputeEffectiveWind() so blocking/wake applies.
             - Choose track_dir = (-1, 0) : move WEST across the arena. */
      const CVector2 wind_eff = ComputeEffectiveWind();  // cm/s
      const Real     Vd       = std::max<Real>(1e-6, m_fBaseCms);
      const CVector2 track_dir(-1.0, 0.0);               // desired ground track (unit-like)

      /* desired forward direction in world frame */
      CVector2 desired_fwd = track_dir - (wind_eff / Vd);
      if(desired_fwd.Length() < 1e-9) {
         desired_fwd = track_dir;  // degenerate case: just face west
      } else {
         desired_fwd.Normalize();
      }

      /* 3) Turn the robot (with wheels) toward desired_fwd */
      const Real desired_yaw = std::atan2(desired_fwd.GetY(), desired_fwd.GetX());
      const Real curr_yaw    = GetYawRadians();

      CRadians err(desired_yaw - curr_yaw);
      err.SignedNormalize();

      /* Very simple P-controller on yaw error to set wheel speeds (cm/s).
         Positive error -> turn left (right wheel forward, left backward), and vice versa. */
      const Real k_turn = 12.0;                // cm/s per radian (tune freely)
      Real turn_cmd = k_turn * err.GetValue(); // cm/s
      /* Saturate to a reasonable turn speed so we don't spin too fast */
      const Real turn_sat = 10.0;              // cm/s
      if(turn_cmd >  turn_sat) turn_cmd =  turn_sat;
      if(turn_cmd < -turn_sat) turn_cmd = -turn_sat;

      if(m_pcWheels) {
         /* In-place rotation (no forward from wheels). Translation will come from impulse. */
         const Real left  = -turn_cmd;
         const Real right =  turn_cmd;
         m_pcWheels->SetLinearVelocity(left, right);
      }

      /* 4) Add the drive impulse along the robot's CURRENT yaw (after we started turning). */
      DriveImpulse(m_fBaseCms);

      /* 5) Schedule post-step to apply (wind + drive) once collisions are resolved */
      HandleAerodynamicsPostStep();
   }
};

/* Register as a separate controller name so it can be used alongside the base one */
REGISTER_CONTROLLER(CWindAwareAirResistance, "wind_aware_air_resistance_controller")

