###############################################################################
# Find openvr
#
# This sets the following variables:
# OpenVR_FOUND - True if openvr was found.
# OpenVR_INCLUDE_DIRS - headers
# OpenVR_LIBRARIES - library

FIND_LIBRARY(OpenVR_LIBRARIES NAMES openvr_api
        PATHS "/home/letrend/openvr/lib/linux64/"
)

FIND_PATH(OpenVR_INCLUDE_DIRS NAMES openvr.h
        HINTS "/home/letrend/openvr/headers/"
)

MARK_AS_ADVANCED(
  OpenVR_INCLUDE_DIRS
  OpenVR_LIBRARIES)

SET( OpenVR_FOUND "NO" )
IF( OpenVR_INCLUDE_DIRS )
  MESSAGE( STATUS "openvr include dirs: ${OpenVR_INCLUDE_DIRS}" )
  INCLUDE_DIRECTORIES( ${OpenVR_INCLUDE_DIRS} )
  IF( OpenVR_LIBRARIES )
    MESSAGE(STATUS "openvr libraries: ${OpenVR_LIBRARIES}" )
    SET( OpenVR_FOUND "YES" )
  ENDIF( OpenVR_LIBRARIES )
ENDIF( OpenVR_INCLUDE_DIRS )

IF( OpenVR_FOUND )
    MESSAGE(STATUS "Found openvr library")
ELSE( OpenVR_FOUND )
    MESSAGE(FATAL_ERROR "Could not find openvr")
ENDIF( OpenVR_FOUND )
