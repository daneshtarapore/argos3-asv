#
# Headers
#
# argos3/plugins/simulator/entities/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_ENTITIES
  box_entity.h
  cylinder_entity.h
  gripper_equipped_entity.h
  led_entity.h
  led_equipped_entity.h
  rab_equipped_entity.h
  wifi_equipped_entity.h
  propeller_equipped_entity.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_ENTITIES
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_ENTITIES}
  box_entity.cpp
  cylinder_entity.cpp
  gripper_equipped_entity.h
  led_entity.cpp
  led_equipped_entity.cpp
  rab_equipped_entity.cpp
  wifi_equipped_entity.cpp
  propeller_equipped_entity.cpp)

#
# Create entity plugin library
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_entities SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_ENTITIES})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_entities argos3plugin_${ARGOS_BUILD_FOR}_media)

#
# Core library installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_ENTITIES} DESTINATION include/argos3/plugins/simulator/entities/)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_entities
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
