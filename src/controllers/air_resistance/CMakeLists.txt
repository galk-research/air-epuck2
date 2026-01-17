add_library(air_resistance MODULE
        air_resistance.cpp
        air_resistance.h)

target_link_libraries(air_resistance
        argos3core_simulator
        argos3plugin_simulator_genericrobot)  # stays generic

# (Optional) If your toolchain needs it for Chipmunk symbols during link:
# target_link_libraries(air_resistance argos3plugin_simulator_dynamics2d)

