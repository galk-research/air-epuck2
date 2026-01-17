# Build loop-functions (logic)
add_library(wind_loop_functions MODULE
        wind_loop_functions.cpp
        wind_loop_functions.h)

target_link_libraries(wind_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_dynamics2d)

set_target_properties(wind_loop_functions PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/lib/loop_functions/wind_loop_functions)

# Build Qt-user-functions (drawing)
add_library(wind_qt_user_functions MODULE
        wind_qt_user_functions.cpp
        wind_qt_user_functions.h)

target_link_libraries(wind_qt_user_functions
        argos3core_simulator
        argos3plugin_simulator_qtopengl)   # << draws the arrow

set_target_properties(wind_qt_user_functions PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/lib/loop_functions/wind_loop_functions)
