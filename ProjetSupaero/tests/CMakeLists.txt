#
# Copyright 2015 CNRS
# Authors : Mathieu Geisert - Florian Valenza
#


MACRO(ADD_EXEC NAME PKGS)
	ADD_EXECUTABLE(${NAME} ${NAME})
  FOREACH(PKG ${PKGS})
	PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)
  TARGET_LINK_LIBRARIES (${NAME} ${PROJECT_NAME})
ENDMACRO(ADD_EXEC)

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

ADD_EXEC(ProjectSupaero "acado;sfml-window;sfml-system;gepetto-viewer-corba;tinyxml2;eigen3")
ADD_EXEC(ProjectSupaero_joystick "acado;sfml-window;sfml-system;gepetto-viewer-corba;tinyxml2;eigen3")
ADD_EXEC(test_viewer_environment "gepetto-viewer-corba;tinyxml2;eigen3")
ADD_EXEC(test_viewer "acado;sfml-window;sfml-system;gepetto-viewer-corba;tinyxml2;eigen3")
ADD_EXEC(test_sfml "sfml-window;sfml-system;sfml-graphics")


ADD_TEST_CFLAGS(ProjectSupaero '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ADD_TEST_CFLAGS(ProjectSupaero_joystick '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ADD_TEST_CFLAGS(test_viewer_environment '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ADD_TEST_CFLAGS(test_viewer '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ADD_TEST_CFLAGS(test_sfml '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
