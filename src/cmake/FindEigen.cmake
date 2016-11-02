#
# Try to find the FreeImage library and include path.
# Once done this will define
#
# EIGEN_FOUND
# EIGEN_INCLUDE_PATH
# EIGEN_LIBRARY
# 

IF (WIN32)
  FIND_PATH( EIGEN_INCLUDE_PATH Eigen/Core
    ${PROJECT_SOURCE_DIR}/extern/eigen3
    DOC "The directory where Eigen/Core resides")
ELSE (WIN32)
  FIND_PATH( BASE_EIGEN_INCLUDE_PATH Eigen/Core
    /usr/include/eigen3
    /usr/local/include/eigen3
    /usr/include
    /usr/local/include
    DOC "The directory where Eigen/Core resides")
ENDIF (WIN32)

SET(EIGEN_INCLUDE_PATH ${BASE_EIGEN_INCLUDE_PATH} ${EIGEN_PLUS_INCLUDE_PATH})
#SET(EIGEN_LIBRARY ${BASE_EIGEN_LIBRARY} ${EIGEN_PLUS_LIBRARY})
#SET(EIGEN_LIBRARIES ${EIGEN_LIBRARY})

IF (EIGEN_INCLUDE_PATH)
  SET( EIGEN_FOUND TRUE CACHE BOOL "Set to TRUE if Eigen/Core is found, FALSE otherwise")
ELSE (EIGEN_INCLUDE_PATH)
  SET( EIGEN_FOUND FALSE CACHE BOOL "Set to TRUE if Eigen/Core is found, FALSE otherwise")
ENDIF (EIGEN_INCLUDE_PATH)

MARK_AS_ADVANCED(
  EIGEN_FOUND 
  EIGEN_INCLUDE_PATH)
