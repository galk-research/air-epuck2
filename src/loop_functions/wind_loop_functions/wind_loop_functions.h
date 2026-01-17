#ifndef WIND_LOOP_FUNCTIONS_H
#define WIND_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector2.h>

/* Logic-only loop-functions (no drawing) */
class CWindLoopFunctions final : public argos::CLoopFunctions {

public:
   void Init(argos::TConfigurationNode& t_node) override;

   /* Accessor for the wind vector (used by Qt user functions) */
   const argos::CVector2& GetWind() const { return m_cWindCms; }

private:
   argos::CVector2 m_cWindCms;   /* wind in cm s⁻¹ */
};

#endif /* WIND_LOOP_FUNCTIONS_H */
