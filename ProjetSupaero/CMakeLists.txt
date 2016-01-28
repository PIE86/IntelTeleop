#
#  Copyright 2015 CNRS
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
SET(CMAKE_CXX_FLAGS "-std=c++14")

project(${PROJECT_NAME})
SETUP_PROJECT()

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("acado")

#SEARCH_FOR_BOOST()
#INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
