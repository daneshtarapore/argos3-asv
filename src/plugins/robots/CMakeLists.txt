add_subdirectory(generic)

if(ARGOS_BUILD_FOR_SIMULATOR OR ARGOS_BUILD_FOR STREQUAL "asv")
  add_subdirectory(asv)
endif(ARGOS_BUILD_FOR_SIMULATOR OR ARGOS_BUILD_FOR STREQUAL "asv")