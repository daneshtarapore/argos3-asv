#
# asv headers
#
# argos3/plugins/robots/asv/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_CONTROLINTERFACE
  control_interface/ci_asv_dualpropeller_actuator.h)
# argos3/plugins/robots/asv/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_SIMULATOR
    simulator/dynamics2d_asv_model.h
    simulator/asv_entity.h
    simulator/asv_dualpropeller_actuator.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# asv sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_CONTROLINTERFACE}
  control_interface/ci_asv_dualpropeller_actuator.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_SIMULATOR}
    simulator/dynamics2d_asv_model.cpp
    simulator/asv_entity.cpp
    simulator/asv_dualpropeller_actuator.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_SIMULATOR}
      simulator/qtopengl_asv.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV}
      simulator/qtopengl_asv.h
      simulator/qtopengl_asv.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create asv plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_asv SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ASV})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_asv
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_asv argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/asv/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ASV_SIMULATOR}      DESTINATION include/argos3/plugins/robots/asv/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_asv
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
