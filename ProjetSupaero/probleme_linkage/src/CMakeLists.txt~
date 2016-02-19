#
# Copyright (c) 2015 CNRS
# Authors: Mathieu Geisert - Florian Valenza
#
#

SET(LIBRARY_NAME ${PROJECT_NAME})


# Source files - Add here your source files
SET(${LIBRARY_NAME}_SOURCES
  viewer.cpp
  environmentparser.cpp
)




SET(${LIBRARY_NAME}_FILES
	${${LIBRARY_NAME}_SOURCES}
	${${LIBRARY_NAME}_HEADERS}
	)


ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} gepetto-viewer-corba)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} tinyxml2)


INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
