#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "depthai::core" for configuration "None"
set_property(TARGET depthai::core APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(depthai::core PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libdepthai-core.so"
  IMPORTED_SONAME_NONE "libdepthai-core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS depthai::core )
list(APPEND _IMPORT_CHECK_FILES_FOR_depthai::core "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libdepthai-core.so" )

# Import target "depthai::opencv" for configuration "None"
set_property(TARGET depthai::opencv APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(depthai::opencv PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libdepthai-opencv.so"
  IMPORTED_SONAME_NONE "libdepthai-opencv.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS depthai::opencv )
list(APPEND _IMPORT_CHECK_FILES_FOR_depthai::opencv "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libdepthai-opencv.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
