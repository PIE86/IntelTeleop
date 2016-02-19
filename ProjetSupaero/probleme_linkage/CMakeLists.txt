#
#  Copyright 2015 CNRS
# Authors: Mathieu Geisert - Florian Valenza
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
#INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME ProjectSupaero_ACADO)
SET(PROJECT_DESCRIPTION "acado for Supaero project : MPC for drones")
SET(PROJECT_URL "")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# project(${PROJECT_NAME})
SETUP_PROJECT()


# Declare Headers -- Add here the header files
SET(${PROJECT_NAME}_HEADERS
  include/viewer.h
  include/environmentparser.h
)

# Dependencies -- Add here dependencies such as sfml, acado, gepetto-viewer-corba
ADD_REQUIRED_DEPENDENCY("gepetto-viewer-corba")
ADD_REQUIRED_DEPENDENCY("tinyxml2")


ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
