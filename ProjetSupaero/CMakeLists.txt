CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
# INCLUDE(cmake/FindSFML.cmake)

SET(PROJECT_NAME ProjectSupaero_ACADO)
# SET(EXECUTABLE_NAME ProjectSupaero)
SET(PROJECT_DESCRIPTION "acado for Supaero project : MPC for drones")
SET(PROJECT_URL "")
SET(PROJECT_VERSION "")

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)
SET(CMAKE_CXX_FLAGS "-std=c++11")

PROJECT(${PROJECT_NAME})
SETUP_PROJECT()
# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("acado")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
