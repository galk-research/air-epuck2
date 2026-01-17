#include "wind_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>

using namespace argos;

/* ---------- read wind once -------------------------------------- */
void CWindLoopFunctions::Init(TConfigurationNode&) {
   TConfigurationNode tAir =
      GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),
                      "configuration"),
              "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(tAir, "angle_deg", deg);
   GetNodeAttribute(tAir, "magnitude", mag);

   const Real rad = deg * ARGOS_PI / 180.0;
   m_cWindCms.Set(mag * std::cos(rad),
                  mag * std::sin(rad));
}

/* ---------- register with ARGoS -------------------------------- */
REGISTER_LOOP_FUNCTIONS(CWindLoopFunctions,
                        "wind_loop_functions")
