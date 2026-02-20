#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ur_coppeliasim::ur_coppeliasim" for configuration ""
set_property(TARGET ur_coppeliasim::ur_coppeliasim APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ur_coppeliasim::ur_coppeliasim PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libur_coppeliasim.so"
  IMPORTED_SONAME_NOCONFIG "libur_coppeliasim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ur_coppeliasim::ur_coppeliasim )
list(APPEND _IMPORT_CHECK_FILES_FOR_ur_coppeliasim::ur_coppeliasim "${_IMPORT_PREFIX}/lib/libur_coppeliasim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
