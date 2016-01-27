FILE(REMOVE_RECURSE
  "CMakeFiles/distdir"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/distdir.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
