file(REMOVE_RECURSE
  "libpinocchio.pdb"
  "libpinocchio.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/pinocchio.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
