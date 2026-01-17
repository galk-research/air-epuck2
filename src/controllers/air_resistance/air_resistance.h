#ifndef AIR_RESISTANCE_H
#define AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector2.h>

/* Forward decl for Chipmunk body (dyn2d) */
struct cpBody;

namespace argos {

   class CAirResistance : public CCI_Controller { /* removed 'final' to allow subclassing */

   public:
      /* All lifecycle methods are virtual so users can override everything if they wish. */
      virtual ~CAirResistance() = default;

      /* Users may fully override Init/ControlStep/Reset/Destroy. */
      virtual void Init(TConfigurationNode& t_node) override;
      virtual void ControlStep()                   override;
      virtual void Reset()                         override {}
      virtual void Destroy()                       override {}

   protected:
      /* physics + wind handling */
      /* These helpers are virtual to allow alternative pipelines in derived classes.
         Derived classes can call the base implementations or replace them entirely. */
      virtual void EnsurePhysicsHandle();              /* obtains dyn2d handle */
      virtual void ApplyWindImpulse();                 /* adds to accumulator */
      virtual void DriveImpulse(Real velocity_cm_s);   /* adds to accumulator */
      virtual void HandleAerodynamicsPreStep();        /* reset + wind + broadcast */
      virtual void HandleAerodynamicsPostStep();       /* schedule single post-step to apply sum */

      /* blocking (returns true if any neighbor provides reduction; sets [0,1]) */
      virtual bool     IsBlockedByRAB(Real& fOutReduction) const;
      virtual CVector2 ComputeEffectiveWind() const;
      virtual Real     GetYawRadians() const;

      /* devices */
      CCI_DifferentialSteeringActuator* m_pcWheels = nullptr; // unused when driving by impulse
      CCI_PositioningSensor*            m_pcPos    = nullptr;
      CCI_RangeAndBearingSensor*        m_pcRABSens= nullptr;
      CCI_RangeAndBearingActuator*      m_pcRABAct = nullptr;

      /* params */
      Real     m_fBaseCms = 5.0f;  /* desired forward speed, cm/s */
      CVector2 m_cWindCms;         /* global wind vector, cm/s */

      /* simple fallback radius broadcast (meters) */
      Real m_fSelfRadiusM = 0.04f;

      /* cached physics */
      bool     m_bBodyReady = false;
      cpBody*  m_ptBody     = nullptr;

      /* per-tick accumulated impulse (world frame) */
      CVector2 m_cAccumImpulse;    // J = (cm/s)/100 * mass * WIND_IMPULSE_SCALE
   };

} /* namespace argos */

#endif /* AIR_RESISTANCE_H */

