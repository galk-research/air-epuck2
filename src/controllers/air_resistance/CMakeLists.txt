# Base implementation compiled once
add_library(air_resistance_base STATIC
  air_resistance.cpp
)

target_include_directories(air_resistance_base PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(air_resistance_base
  argos3core_simulator
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_dynamics2d
)

# ARGoS module that registers the base controller
add_library(air_resistance MODULE
  air_resistance_register.cpp
  air_resistance.h
)

target_link_libraries(air_resistance
  air_resistance_base
)

set_target_properties(air_resistance PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/controllers/air_resistance
)

