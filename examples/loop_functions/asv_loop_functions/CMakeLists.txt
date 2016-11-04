link_directories(${CMAKE_BINARY_DIR}/controllers/asv_test)
set(asv_loop_functions_SOURCES asv_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(asv_loop_functions_SOURCES
    ${asv_loop_functions_SOURCES}
    asv_qt_user_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(asv_loop_functions MODULE ${asv_loop_functions_SOURCES})
target_link_libraries(asv_loop_functions
  asv_test
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_asv
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(asv_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)
