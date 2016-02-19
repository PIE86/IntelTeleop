FILE(REMOVE_RECURSE
  "doc/ProjectSupaero_ACADO.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/distorig"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/distorig.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
