#
#  Copyright 2015 CNRS
# Authors : Mathieu Geisert - Florian Valenza
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
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

ADD_EXEC(test1 "gepetto-viewer-corba") # if multiple external libraries use ; to list them >>  "gepetto-viewer-corba;sfml;acado"
ADD_TEST_CFLAGS(test1 '-DPIE_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
