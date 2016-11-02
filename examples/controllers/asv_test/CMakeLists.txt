add_library(asv_test MODULE asv_test.h asv_test.cpp)
target_link_libraries(asv_test
  argos3core_simulator
  argos3plugin_simulator_asv
  argos3plugin_simulator_genericrobot)
