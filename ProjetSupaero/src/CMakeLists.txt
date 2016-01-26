#
# Copyright (c) 2015 CNRS
# Authors: Mathieu Geisert
#
#

SET(LIBRARY_NAME ${PROJECT_NAME})

#SET( SOURCES
#  ProjectSupaero
#  quadModel
#  quadRungeKutta
#  systemEvol
#  controlMPC
#  )

#FOREACH( source ${SOURCES} )
#  ADD_EXECUTABLE( ${source} ${source}.cpp)
#  PKG_CONFIG_USE_DEPENDENCY(${source} acado)
#  INSTALL(TARGETS ${source} DESTINATION bin)
#ENDFOREACH(source)


# Modified by JIA Zhengxuan
# To add all the source files together to the executable list,
# there are two means:
# (1) using a global file
# (2) storing the names of all the source files into a variable

# (1) By the first method
# FILE(GLOB Srcs *.cpp)
# ADD_EXECUTABLE(${PROJECT_NAME} ${Srcs})

# (2) By the second method
AUX_SOURCE_DIRECTORY(. SRCS)
# in add_executable there are two parameters
# the first indicates the name of the executable file
# the second is the file to be compiled into executable file
ADD_EXECUTABLE(ProjectSupaero ${SRCS})

PKG_CONFIG_USE_DEPENDENCY(ProjectSupaero acado)
INSTALL(TARGETS ProjectSupaero DESTINATION bin)


















