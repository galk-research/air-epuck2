add_library(wind_aware_air_resistance MODULE
  wind_aware_air_resistance.cpp
)

target_include_directories(wind_aware_air_resistance PRIVATE
  ${CMAKE_SOURCE_DIR}/src/controllers/air_resistance
)

target_link_libraries(wind_aware_air_resistance
  air_resistance_base
  argos3core_simulator
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_dynamics2d
)

set_target_properties(wind_aware_air_resistance PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/controllers/wind_aware
)

