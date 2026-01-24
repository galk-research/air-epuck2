add_library(formation_template MODULE
  formation_template.cpp
)

target_include_directories(formation_template PRIVATE
  ${CMAKE_SOURCE_DIR}/src/controllers/air_resistance
)

# If you already created air_resistance_base (recommended), link it:
target_link_libraries(formation_template
  air_resistance_base
  argos3core_simulator
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_dynamics2d
)

set_target_properties(formation_template PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/controllers/formation_template
)

